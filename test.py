# -*- coding: utf-8 -*-
import serial
import threading
import json
import time

# 시리얼 포트와 보레이트 설정
SERIAL_PORT = 'COM21'  # 시리얼 포트 이름을 실제 사용하는 포트로 변경
BAUD_RATE = 115200

# 시리얼 포트 열기
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

def read_from_port(ser):
    while True:
        if ser.in_waiting > 0:
            try:
                data = ser.readline().decode('utf-8').rstrip()
                print(data)
                json_data = json.loads(data)
                print(f"Received JSON: {json.dumps(json_data, indent=4)}")
            except json.JSONDecodeError:
                print(f"Received non-JSON data: {data}")
            except Exception as e:
                print(f"Error: {e}")

def write_to_port(ser, jcmd):
    try:
        ser.write((json.dumps(jcmd) + '\r\n').encode('utf-8'))
        ser.flush()
    except json.JSONDecodeError:
        print("Invalid JSON format. Please enter valid JSON.")
    except Exception as e:
        print(f"Error: {e}")

# 읽기 스레드 시작
read_thread = threading.Thread(target=read_from_port, args=(ser,))
read_thread.daemon = True
read_thread.start()

if __name__ == "__main__":

    jcmd = {"cmd":"set_mtsp", "mt0": 00, "mt1": 0 }
    #jcmd = {"cmd":"servo_ang", "ang": 10 }
    
    write_to_port(ser, jcmd)

    while True:
        
        # jcmd = {"cmd":"get_cnt"}
        # write_to_port(ser, jcmd)
        
        time.sleep(1)
        print("loop")
