import serial
import time

# 아두이노와 연결된 포트 설정
arduino = serial.Serial('/dev/ttyUSB0', 115200)  # 포트와 속도 확인

# 아두이노가 준비될 때까지 기다리기
time.sleep(2)

# 메시지를 보내기 전에 출력
msg = "100,200,300\n"
print(f"Sending: {msg.strip()}")  # 전송할 메시지 출력

# 아두이노에 메시지 보내기
arduino.write(msg.encode())

# 아두이노에서 응답받기
response = arduino.readline()
print(f"Response from Arduino: {response.decode().strip()}")

# 연결 종료
arduino.close()
