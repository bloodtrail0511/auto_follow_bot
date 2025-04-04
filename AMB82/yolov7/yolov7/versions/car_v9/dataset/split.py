import os
import shutil
from sklearn.model_selection import train_test_split
import glob

# 設定圖片和標籤文件的來源目錄
source_image_dir = 'car_v9/dataset/source/images'  # 更新此路徑為實際的圖片文件目錄
source_label_dir = 'car_v9/dataset/source/labels'  # 更新此路徑為實際的標籤文件目錄
train_image_dir = 'car_v9/dataset/train/images'
# test_image_dir = 'car_v9/dataset/test/images'
valid_image_dir = 'car_v9/dataset/valid/images'
train_label_dir = 'car_v9/dataset/train/labels'
# test_label_dir = 'car_v9/dataset/test/labels'
valid_label_dir = 'car_v9/dataset/valid/labels'

# 創建目錄（如果不存在）
os.makedirs(train_image_dir, exist_ok=True)
# os.makedirs(test_image_dir, exist_ok=True)
os.makedirs(train_label_dir, exist_ok=True)
# os.makedirs(test_label_dir, exist_ok=True)
os.makedirs(valid_image_dir, exist_ok=True)
os.makedirs(valid_label_dir, exist_ok=True)

# 列出所有圖片文件
image_paths = glob.glob(os.path.join(source_image_dir, '*'))

train_paths, valid_paths = train_test_split(image_paths, test_size=0.2, random_state=42)


# # 将图片文件首先分为 60% 训练集，40% 剩余数据
# train_paths, remaining_paths = train_test_split(image_paths, test_size=0.4, random_state=42)

# # 将剩余的 40% 数据再分为 50% 验证集，50% 测试集（相当于原数据的 20%）
# valid_paths, test_paths = train_test_split(remaining_paths, test_size=0.5, random_state=42)

# 移动图片和标签到训练集、验证集和测试集目录
def move_files(paths, image_dir, label_dir):
    for path in paths:
        shutil.move(path, os.path.join(image_dir, os.path.basename(path)))
        label_path = os.path.join(source_label_dir, os.path.basename(path).replace('.jpg', '.txt'))
        if os.path.exists(label_path):
            shutil.move(label_path, os.path.join(label_dir, os.path.basename(label_path)))

move_files(train_paths, train_image_dir, train_label_dir)
move_files(valid_paths, valid_image_dir, valid_label_dir)
# move_files(test_paths, test_image_dir, test_label_dir)

print(f'Moved {len(train_paths)} images and labels to {train_image_dir} and {train_label_dir}')
print(f'Moved {len(valid_paths)} images and labels to {valid_image_dir} and {valid_label_dir}')
# print(f'Moved {len(test_paths)} images and labels to {test_image_dir} and {test_label_dir}')