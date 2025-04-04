import serial
import math
import csv
import time

PORT = "COM3"                # 串列通訊埠
BAUD_RATE = 115200           # 波特率
OUTPUT_FILE = "./橢球擬合/long_time_angle_error.csv"  # 輸出檔案路徑
RUN_DURATION = 30*60        # 30min

ser = serial.Serial(PORT, BAUD_RATE)  # 初始化串列通訊

try:
    with open(OUTPUT_FILE, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["fused", "gyro", "mag"])  # 寫入標題行

        print("開始接收數據，按下 Ctrl+C 停止...")
        start_time = time.time()
        while True:
            # 讀取串列資料
            try:
                X, Y, Z = map(float, ser.readline().decode().split(","))
                print(f"{X:.4f}, {Y:.4f}, {Z:.4f}")

                # 即時寫入 CSV 文件
                writer.writerow([X, Y, Z])
                if(time.time() - start_time > RUN_DURATION):
                    print(f"已達到 30 分鐘，自動停止收集數據。")
                    break

            except ValueError:
                # 跳過解析錯誤的行
                print("數據解析錯誤，跳過一行。")

except KeyboardInterrupt:
    print("\n捕捉到 Ctrl+C，已停止接收數據。")
finally:
    ser.close()  # 關閉串列連接
    print("串列連接已關閉。")
    print(f"數據已保存到 {OUTPUT_FILE}")
