import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# ================== GPIO 初始化 ==================
PWMA, AIN1, AIN2 = 18, 22, 27
PWMB, BIN1, BIN2 = 23, 25, 24

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

for pin in [AIN1, AIN2, PWMA, BIN1, BIN2, PWMB]:
    GPIO.setup(pin, GPIO.OUT)

L_Motor = GPIO.PWM(PWMA, 50)
R_Motor = GPIO.PWM(PWMB, 50)
L_Motor.start(0)
R_Motor.start(0)

# ================== PID 控制器（轻量）==================
class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.reset()

    def reset(self):
        self.last_error = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        if dt <= 0:
            return 0
        self.integral += error * dt
        self.integral = max(min(self.integral, 50), -50)  # 更紧积分限
        derivative = (error - self.last_error) / dt
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

pid_dist = PID(0.1, 0.0005, 0.002)
pid_turn = PID(0.8, 0.001, 0.003)

# ================== 摄像头设置 ==================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 关键：减少缓冲帧

FRAME_CENTER_X = 160
TARGET_AREA = 4000

# ================== 平滑参数 ==================
alpha = 0.6
last_x, last_area = FRAME_CENTER_X, 0

# ================== 时间控制 ==================
last_time = time.perf_counter()

# ================== 电机控制 ==================
def drive(left, right):
    left = np.clip(left, 0, 100)
    right = np.clip(right, 0, 100)
    L_Motor.ChangeDutyCycle(left)
    R_Motor.ChangeDutyCycle(right)
    # 正转方向（根据你硬件确认）
    GPIO.output(AIN1, True); GPIO.output(AIN2, False)
    GPIO.output(BIN1, True); GPIO.output(BIN2, False)

def back(speed=30):
    speed = np.clip(speed, 0, 100)
    L_Motor.ChangeDutyCycle(speed)
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN1, False); GPIO.output(AIN2, True)
    GPIO.output(BIN1, False); GPIO.output(BIN2, True)

def stop():
    L_Motor.ChangeDutyCycle(0)
    R_Motor.ChangeDutyCycle(0)

# ================== 颜色检测（内存优化）==================
def detect_color(frame):
    global last_x, last_area

    # 直接在原图上模糊（避免 copy）
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # 固定绿色范围（可调）
    mask = cv2.inRange(hsv, (35, 80, 80), (85, 255, 255))

    # 简化形态学：只做一次开运算（去噪）
    kernel = np.ones((3, 3), np.uint8)  # ⬇️ 减小核尺寸
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        cv2.imshow("Mask", mask)  # 仍显示用于调试
        return False, 0, 0, frame

    # 找最大轮廓
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < 400:
        cv2.imshow("Mask", mask)
        return False, 0, 0, frame

    x, y, w, h = cv2.boundingRect(largest)
    cx = x + w // 2

    # 平滑
    cx = int(alpha * cx + (1 - alpha) * last_x)
    area = alpha * area + (1 - alpha) * last_area
    last_x, last_area = cx, area

    # 画框（仅当需要显示时）
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 1)
    cv2.circle(frame, (cx, y + h // 2), 3, (0, 0, 255), -1)

    cv2.imshow("Mask", mask)
    return True, cx, area, frame

# ================== 主控制逻辑 ==================
def follow(frame):
    global last_time

    found, x, area, frame = detect_color(frame)

    if not found:
        stop()
        cv2.putText(frame, "No Target", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)
        return frame

    now = time.perf_counter()
    dt = now - last_time
    last_time = now
    if dt > 0.1:  # 防止异常大 dt
        dt = 0.01

    # PID 计算
    error_dist = TARGET_AREA - area
    speed_out = pid_dist.update(error_dist, dt)

    error_turn = x - FRAME_CENTER_X
    turn_out = pid_turn.update(error_turn, dt)

    base_speed = 15
    left = base_speed + speed_out + turn_out
    right = base_speed + speed_out - turn_out

    # 防撞
    if area > 8000:
        back(40)
        cv2.putText(frame, "BACK!", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)
        return frame

    drive(left, right)

    # UI 显示（精简）
    cv2.line(frame, (FRAME_CENTER_X, 0), (FRAME_CENTER_X, 240), (255, 0, 0), 1)
    cv2.putText(frame, f"Area:{int(area)}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    cv2.putText(frame, f"L:{int(left)} R:{int(right)}", (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    return frame

# ================== 主循环 ==================
print("=== 颜色跟随启动（内存优化版）===")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        frame = follow(frame)

        #  降低显示频率（可选）：每2帧显示一次
        # if int(time.time() * 10) % 2 == 0:
        cv2.imshow("Follow", frame)

        if cv2.waitKey(1) == 27:  # ESC 退出
            break

except KeyboardInterrupt:
    pass

finally:
    print("清理资源...")
    stop()
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()