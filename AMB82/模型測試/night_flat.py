import os
import matplotlib.pyplot as plt
plt.rcParams["font.family"] = "DFKai-SB"
# 假設資料夾名稱為 'data_files' 且與 Python 檔在同一目錄
folder_path = './夜晚跟隨測試(平坦路面)'

# 存放每個檔案的成功率
success_rates = {}

# 讀取資料夾中的檔案
for filename in os.listdir(folder_path):
    if filename.endswith('.txt'):  # 確保只讀取 .txt 檔案
        file_path = os.path.join(folder_path, filename)
        with open(file_path, 'r') as file:
            # 讀取檔案內容，移除逗號並計算成功率
            content = file.read().replace(",", "").strip()
            total_chars = len(content)
            success_count = content.count('1')
            success_rate = success_count / total_chars
            success_rates[filename] = success_rate

# 取成功率最低的七個檔案
sorted_success_rates = dict(sorted(success_rates.items(), key=lambda x: x[1]))
lowest_seven = dict(list(sorted_success_rates.items())[:7])

# 繪製長條圖
plt.figure(figsize=(12, 7))
bars = plt.bar(range(len(lowest_seven)), lowest_seven.values())  # 加寬長條

# 在長條圖上顯示百分比
for bar in bars:
    height = bar.get_height()
    plt.text(bar.get_x() + bar.get_width() / 2, height, f'{height:.2%}', 
             ha='center', va='bottom', fontsize=10)

# plt.xlabel('File Names', fontsize=14)
plt.ylabel('跟隨成功率', fontsize=14)
plt.title('夜晚跟隨測試(平坦路面)', fontsize=16)
# plt.xticks(rotation=45, ha='right', fontsize=12)
# plt.yticks(fontsize=12)
plt.tight_layout()
print(f"{sum(lowest_seven.values())/7*100}")
# plt.savefig("夜晚跟隨測試(平坦路面).png")
# plt.show()