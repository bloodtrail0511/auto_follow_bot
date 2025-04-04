import serial
import math
import csv
import numpy as np

PORT = "COM4"
BAUD_RATE = 115200
# OUTPUT_FILE = "./橢球擬合/data_only_esp.csv"
# OUTPUT_FILE = "./橢球擬合/data.csv"
OUTPUT_FILE = "./data2.csv"
# OUTPUT_FILE = "./橢球擬合/roll_xyangle2.csv"
# OUTPUT_FILE = "./橢球擬合/pitch_xyangle2.csv"

data = []

save_data = True

ser = serial.Serial(PORT, BAUD_RATE)

try:
    while True:
        try:
            # 讀取數據並嘗試解析
            line = ser.readline().decode().strip()  # 去除空白字符
            X, Y, Z = map(float, line.split(","))
            print(f"{X:.4f}, {Y:.4f}, {Z:.4f}")

            # 如果開啟保存模式，將數據加入列表
            if save_data:
                data.append([X, Y, Z])
        except ValueError as e:
            # 忽略無法解析的行並打印錯誤訊息
            print(f"跳過無法解析的數據行: {line} ({e})")
        except Exception as e:
            # 捕捉其他潛在錯誤
            print(f"未知錯誤: {e} (忽略並繼續)")

except KeyboardInterrupt:
    if save_data:
        print("\n捕捉到 Ctrl+C，正在保存數據...")
        # 將數據寫入 CSV 文件
        with open(OUTPUT_FILE, mode='w', newline='') as file:
            writer = csv.writer(file)
            # 寫入標題
            writer.writerow(["x", "y", "z"])
            # 寫入數據
            writer.writerows(data)
        
        print(f"數據已保存到 {OUTPUT_FILE}")

finally:
    ser.close()  # 關閉串列連接
    print("串列連接已關閉。")
