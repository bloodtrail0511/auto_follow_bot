import serial


PORT = "COM3"
BAUD_RATE = 115200
ser = serial.Serial(PORT, BAUD_RATE)

try:
    while True:
        angle = int(ser.readline())
except KeyboardInterrupt:
    print("已中斷程式")
finally:
    print("執行結束")