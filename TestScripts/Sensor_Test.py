import time
from ProjectPath import PROJECT_PATH
from ClassFiles.MagnetFieldSensor import MagnetFieldSensor
from pynput import keyboard

GDX = MagnetFieldSensor()


# 👉 키 입력 처리 함수
def on_press(key):
    try:
        if key == keyboard.Key.space:  # 스페이스바 → 함수 실행
            print("Sensing...")
            data = GDX.Measure()
            data = [round(x, 3) for x in data]

            print(f"Data: {data} [mT]")
            print("")

        elif key == keyboard.Key.esc:  # ESC → 종료
            print("🛑 프로그램 종료")
            return False  # Listener 종료
    except Exception as e:
        print(f"에러 발생: {e}")


if GDX.Connect == True:
    # 👉 메인 루프
    print("===== 키 입력 대기 중 =====")
    print("스페이스바 → 측정 실행")
    print("ESC → 종료\n")
    print("")
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

