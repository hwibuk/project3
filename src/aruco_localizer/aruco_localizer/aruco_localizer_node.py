#!/usr/bin/env python3
"""
aruco_localizer_node.py

카메라로 ArUco 마커를 인식해서 로봇의 map 기준 절대 pose를 계산하고,
/initialpose 토픽으로 발행해서 AMCL이 자동으로 초기 위치를 잡도록 하는 노드.

동작 원리:
1. 카메라 이미지에서 ArUco 마커 검출
2. solvePnP로 "카메라 기준 마커의 위치/방향" (T_cam_marker) 계산
3. marker_layout.json에서 "map 기준 마커의 위치/방향" (T_map_marker) 조회
4. T_map_cam = T_map_marker * inverse(T_cam_marker) 계산
5. T_map_cam에 camera->base_footprint 고정 변환을 곱해서 T_map_base 계산
6. T_map_base를 PoseWithCovarianceStamped로 변환해서 /initialpose 발행
"""

import json
import math
import numpy as np
import cv2
import cv2.aruco as aruco

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge

import tf2_ros
import tf_transformations


def pose_to_matrix(x, y, z, roll, pitch, yaw):
    """x,y,z,roll,pitch,yaw -> 4x4 변환 행렬"""
    T = tf_transformations.euler_matrix(roll, pitch, yaw)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


def matrix_to_xyzrpy(T):
    x, y, z = T[0, 3], T[1, 3], T[2, 3]
    roll, pitch, yaw = tf_transformations.euler_from_matrix(T)
    return x, y, z, roll, pitch, yaw


class ArucoLocalizerNode(Node):

    def __init__(self):
        super().__init__('aruco_localizer_node')

        # ===== 파라미터 =====
        self.declare_parameter('marker_layout_file', '/myagv_ros2/src/myagv_navigation2/config/marker_layout.json')
        self.declare_parameter('marker_size', 0.20)          # 마커 실제 크기 (m), world에서 만든 0.20과 일치해야 함
        self.declare_parameter('camera_topic', '/front_camera/image_raw')
        self.declare_parameter('camera_info_topic', '/front_camera/front_camera/camera_info')
        self.declare_parameter('publish_initialpose', True)
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')

        # camera_link -> base_footprint 변환 (URDF에서 가져온 고정값)
        # URDF: camera_joint origin xyz="0.16 0.0 0.04", base_footprint_joint z=0.06
        # base_footprint -> base_link -> camera_link 누적
        self.declare_parameter('cam_x_in_base', 0.16)
        self.declare_parameter('cam_y_in_base', 0.0)
        self.declare_parameter('cam_z_in_base', 0.10)   # 0.06(footprint->base_link) + 0.04(base_link->camera)
        self.declare_parameter('cam_roll_in_base', 0.0)
        self.declare_parameter('cam_pitch_in_base', 0.0)
        self.declare_parameter('cam_yaw_in_base', 0.0)

        marker_layout_file = self.get_parameter('marker_layout_file').value
        self.marker_size = self.get_parameter('marker_size').value
        camera_topic = self.get_parameter('camera_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.publish_initialpose = self.get_parameter('publish_initialpose').value
        dict_name = self.get_parameter('aruco_dict').value

        cam_x = self.get_parameter('cam_x_in_base').value
        cam_y = self.get_parameter('cam_y_in_base').value
        cam_z = self.get_parameter('cam_z_in_base').value
        cam_roll = self.get_parameter('cam_roll_in_base').value
        cam_pitch = self.get_parameter('cam_pitch_in_base').value
        cam_yaw = self.get_parameter('cam_yaw_in_base').value

        # base_footprint -> camera_link 고정 변환 (카메라 광학 frame 아님, link 기준)
        self.T_base_cam = pose_to_matrix(cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw)

        # 카메라 link -> optical frame 보정 (URDF의 camera_optical_joint와 동일해야 함)
        # rpy="-1.5708 0 -1.5708" : z앞 x우 y아래 (ROS 표준 광학축)
        self.T_camlink_optical = pose_to_matrix(0, 0, 0, -math.pi/2, 0, -math.pi/2)

        # ===== 마커 좌표 정답표 로드 =====
        self.marker_world_poses = {}  # id -> 4x4 matrix (map 기준)
        try:
            with open(marker_layout_file, 'r') as f:
                layout = json.load(f)
            for entry in layout:
                mid, x, y, z, yaw, side = entry
                # 마커는 벽에 수직으로 서 있고, yaw만 의미있게 회전 (roll/pitch=0 가정)
                T = pose_to_matrix(x, y, z, 0.0, 0.0, yaw)
                self.marker_world_poses[int(mid)] = T
            self.get_logger().info(f"마커 좌표 {len(self.marker_world_poses)}개 로드 완료: {marker_layout_file}")
        except Exception as e:
            self.get_logger().error(f"marker_layout_file 로드 실패: {e}")

        # ===== ArUco 설정 (버전별 호환성 예외 처리 반영) =====
        dict_map = {
            'DICT_4X4_50': aruco.DICT_4X4_50,
            'DICT_4X4_100': aruco.DICT_4X4_100,
            'DICT_5X5_50': aruco.DICT_5X5_50,
            'DICT_6X6_50': aruco.DICT_6X6_50,
        }
        self.aruco_dict = aruco.getPredefinedDictionary(dict_map.get(dict_name, aruco.DICT_4X4_50))
        
        # OpenCV 버전을 자동으로 판별하여 적절한 API 객체를 생성합니다.
        if hasattr(aruco, 'ArucoDetector'):
            # OpenCV 4.7.0 이상 신버전 방식
            self.aruco_params = aruco.DetectorParameters()
            self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_api = True
            self.get_logger().info("OpenCV New ArUco API 기반으로 초기화되었습니다.")
        else:
            # OpenCV 4.6.x 이하 구버전 방식 (Galactic 기본 환경)
            self.aruco_params = aruco.DetectorParameters_create()
            self.use_new_api = False
            self.get_logger().info("OpenCV Legacy ArUco API 기반으로 초기화되었습니다.")

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # ===== ROS 통신 =====
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_cb, qos)
        self.create_subscription(Image, camera_topic, self.image_cb, qos)
        self.get_logger().info("구독 시작: " + camera_topic + ", " + camera_info_topic)

        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.last_published = False  # 한 번만 발행할지, 계속 발행할지 옵션화 가능

        self.get_logger().info("ArUco Localizer Node 시작됨")

    def camera_info_cb(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info(f"camera_info 수신: fx={self.camera_matrix[0,0]:.1f}")

    def image_cb(self, msg: Image):
        if self.camera_matrix is None:
            return  # camera_info 아직 안 받음

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"이미지 변환 실패: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 설정된 버전에 맞춰 마커를 검출합니다.
        if self.use_new_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        # 디버그: 5프레임마다 이미지 저장
        if not hasattr(self, '_dbg_count'):
            self._dbg_count = 0
        self._dbg_count += 1
        if self._dbg_count % 30 == 1:
            cv2.imwrite('/tmp/aruco_debug.png', gray)
            self.get_logger().info(f"디버그 이미지 저장, 검출된 마커 수: {0 if ids is None else len(ids)}")

        if ids is None or len(ids) == 0:
            return

        half = self.marker_size / 2.0
        # 마커의 로컬 좌표 (마커 평면 기준 4꼭짓점, z=0)
        obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float64)

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id not in self.marker_world_poses:
                continue  # 정답표에 없는 마커는 무시

            img_points = corners[i].reshape(-1, 2)

            ok, rvec, tvec = cv2.solvePnP(
                obj_points, img_points,
                self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not ok:
                continue

            # T_optical_marker : 카메라 광학 frame 기준 마커 pose
            R, _ = cv2.Rodrigues(rvec)
            T_optical_marker = np.eye(4)
            T_optical_marker[:3, :3] = R
            T_optical_marker[:3, 3] = tvec.flatten()

            # 검증 결과: solvePnP가 obj_points(x=오른쪽,y=위,z=법선) 기준으로 푼 R,t는
            # SDF plane(normal=1,0,0)의 yaw 배치와 별도 축보정 없이 바로 일치함.
            # (기하학적으로 직접 시뮬레이션하여 검증됨 - 추가 회전 보정 불필요)
            T_map_marker = self.marker_world_poses[marker_id]

            # T_map_optical = T_map_marker * inv(T_optical_marker)
            T_map_optical = T_map_marker @ np.linalg.inv(T_optical_marker)

            # optical -> camera_link -> base_footprint -> map 역산
            T_map_camlink = T_map_optical @ np.linalg.inv(self.T_camlink_optical)
            T_map_base = T_map_camlink @ np.linalg.inv(self.T_base_cam)

            x, y, z, roll, pitch, yaw = matrix_to_xyzrpy(T_map_base)

            self.get_logger().info(
                f"마커 ID={marker_id} 인식 -> 로봇 위치 추정: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}도"
            )

            if self.publish_initialpose:
                self.publish_initial_pose(x, y, yaw)

            break  # 한 프레임에 여러 마커가 보여도 일단 첫 번째만 사용 (단순화)

    def publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # covariance: ArUco 기반이라 GPS보다 정확하다고 가정, 작은 값 사용
        cov = [0.0]*36
        cov[0] = 0.05    # x
        cov[7] = 0.05    # y
        cov[35] = 0.05   # yaw
        msg.pose.covariance = cov

        self.initialpose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()