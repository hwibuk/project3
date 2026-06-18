import sys
from gtts import gTTS
import os
import pygame
import time

def speak_korean():
    # pygame의 오디오 모듈 초기화
    pygame.mixer.init()

    print("--- 한글 TTS 프로그램 (종료하려면 '종료' 입력) ---")

    while True:
        # sys.stdin을 사용하여 입력을 UTF-8로 강제 처리
        print("\n읽어줄 내용을 입력하세요: ", end='', flush=True)
        text = sys.stdin.readline().strip()

        if text == "종료":
            print("프로그램을 종료합니다.")
            break

        if not text:
            continue

        try:
            # 1. gTTS로 변환
            tts = gTTS(text=text, lang='ko')
            filename = "temp_voice.mp3"
            tts.save(filename)

            # 2. pygame으로 재생
            pygame.mixer.music.load(filename)
            pygame.mixer.music.play()

            # 재생이 끝날 때까지 대기
            while pygame.mixer.music.get_busy():
                time.sleep(0.1)

            # 3. 파일 정리
           # pygame.mixer.music.unload()
           # os.remove(filename)

        except Exception as e:
            print(f"오류가 발생했습니다: {e}")

if __name__ == "__main__":
    speak_korean()
