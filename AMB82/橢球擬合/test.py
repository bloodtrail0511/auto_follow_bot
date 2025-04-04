########################################################################################################################3
import pygame
import math
import serial

# ====== Serial 通訊設定 ======
try:
    ser = serial.Serial('COM3', 115200, timeout=1)
    print("Serial connection established.")
except Exception as e:
    print("Error:", e)
    exit()

# ====== Pygame 設定 ======
pygame.init()
screen = pygame.display.set_mode((700, 800))
pygame.display.set_caption("IMU Angle Visualization")
clock = pygame.time.Clock()
# 顏色設定
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)
ORANGE = (255, 165, 0)

# 中心點與半徑
center = (350, 400)
radius = 300

# 字體設定
font = pygame.font.SysFont(None, 24)

# ====== 繪製圖例 ======
def draw_legend():
    pygame.draw.rect(screen, WHITE, (10, 10, 180, 60))  # 背景框
    pygame.draw.line(screen, BLUE, (20, 30), (40, 30), 3)  # 未補償顯示條
    pygame.draw.line(screen, ORANGE, (20, 50), (40, 50), 3)  # 補償顯示條

    # 添加文字
    uncompensated_label = font.render("Uncompensated", True, BLACK)
    compensated_label = font.render("Compensated", True, BLACK)
    screen.blit(uncompensated_label, (50, 20))
    screen.blit(compensated_label, (50, 40))

# ====== 繪製圓上的角度 ======
# def draw_circle_with_angles():
#     pygame.draw.circle(screen, BLACK, center, radius, 2)
#     for angle in range(0, 360, 30):  # 每 30 度標記一次
#         rad = math.radians(angle - 90)  # 將角度調整為 0 度朝上
#         x = center[0] + (radius + 20) * math.cos(rad)
#         y = center[1] + (radius + 20) * math.sin(rad)
#         angle_text = font.render(f"{angle}°", True, BLACK)
#         screen.blit(angle_text, (x - angle_text.get_width() / 2, y - angle_text.get_height() / 2))
def draw_circle_with_angles():
    pygame.draw.circle(screen, BLACK, center, radius, 2)
    for angle in range(0, 360, 30):  # 每 30 度標記一次
        rad = math.radians(angle - 90)  # 將角度調整為 0 度朝上
        # 外圓刻度的終點
        outer_x = center[0] + radius * math.cos(rad)
        outer_y = center[1] + radius * math.sin(rad)
        # 內圓刻度的起點
        inner_x = center[0] + (radius - 10) * math.cos(rad)
        inner_y = center[1] + (radius - 10) * math.sin(rad)

        # 繪製刻度線
        pygame.draw.line(screen, BLACK, (inner_x, inner_y), (outer_x, outer_y), 2)

        # 在外圓的稍遠位置繪製角度文字
        text_x = center[0] + (radius + 20) * math.cos(rad)
        text_y = center[1] + (radius + 20) * math.sin(rad)
        angle_text = font.render(f"{angle}°", True, BLACK)
        screen.blit(angle_text, (text_x - angle_text.get_width() / 2, text_y - angle_text.get_height() / 2))


# ====== 主迴圈 ======
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    try:
        line = ser.readline().decode('utf-8').strip()
        if line:
            # 假設數據格式為 "uncompensated_angle,compensated_angle"
            uncompensated_angle, compensated_angle = map(float, line.split(','))

            # 將角度轉換為弧度，並調整 0 度朝上（90 度偏移）
            uncompensated_angle_rad = math.radians(uncompensated_angle - 90)
            compensated_angle_rad = math.radians(compensated_angle - 90)

            # 計算箭頭的位置 (XY 平面)
            uncompensated_x = center[0] + radius * math.cos(uncompensated_angle_rad)
            uncompensated_y = center[1] + radius * math.sin(uncompensated_angle_rad)
            compensated_x = center[0] + radius * math.cos(compensated_angle_rad)
            compensated_y = center[1] + radius * math.sin(compensated_angle_rad)

            # 繪製背景
            screen.fill(WHITE)

            # 繪製圓與角度標記
            draw_circle_with_angles()

            # 繪製箭頭
            pygame.draw.line(screen, BLUE, center, (uncompensated_x, uncompensated_y), 3)
            pygame.draw.line(screen, ORANGE, center, (compensated_x, compensated_y), 3)

            # 繪製圖例
            draw_legend()

            # 更新畫面
            pygame.display.flip()
            clock.tick(500)

            print(f"Uncompensated Angle: {uncompensated_angle:.2f}°, Compensated Angle: {compensated_angle:.2f}°")

    except ValueError:
        print("Error: Invalid data format. Ensure ESP32 sends 'uncompensated_angle,compensated_angle'.")

pygame.quit()
ser.close()
