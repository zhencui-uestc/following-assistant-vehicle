import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# ================== GPIO ==================
PWMA, AIN1, AIN2 = 18, 22, 27
PWMB, BIN1, BIN2 = 23, 25, 24

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

for pin in [AIN1, AIN2, PWMA, BIN1, BIN2, PWMB]:
    GPIO.setup(pin, GPIO.OUT)

L = GPIO.PWM(PWMA, 100)
R = GPIO.PWM(PWMB, 100)
L.start(0)
R.start(0)

# ================== 电机 ==================
def forward(l, r):
    l = np.clip(l, 0, 100)
    r = np.clip(r, 0, 100)
    L.ChangeDutyCycle(l)
    R.ChangeDutyCycle(r)
    GPIO.output(AIN1, 1); GPIO.output(AIN2, 0)
    GPIO.output(BIN1, 1); GPIO.output(BIN2, 0)

def backward(l, r):
    l = np.clip(l, 0, 100)
    r = np.clip(r, 0, 100)
    L.ChangeDutyCycle(l)
    R.ChangeDutyCycle(r)
    GPIO.output(AIN1, 0); GPIO.output(AIN2, 1)
    GPIO.output(BIN1, 0); GPIO.output(BIN2, 1)

def stop():
    L.ChangeDutyCycle(0)
    R.ChangeDutyCycle(0)

# ================== PID（只用于转向）==================
class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.last = 0
        self.i = 0

    def update(self, e, dt):
        if dt <= 0:
            return 0
        self.i += e * dt
        self.i = np.clip(self.i, -50, 50)
        d = (e - self.last) / dt
        self.last = e
        return self.kp*e + self.ki*self.i + self.kd*d

pid_turn = PID(0.6, 0, 0.002)

# ================== 摄像头 ==================
cap = cv2.VideoCapture(0)
cap.set(3, 320)
cap.set(4, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

CENTER = 160

# ================== 平滑 ==================
alpha = 0.6
last_x = CENTER
last_area = 0
last_speed = 0

# ================== 颜色检测 ==================
def detect(frame):
    global last_x, last_area

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (35, 80, 80), (85, 255, 255))

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return False, 0, 0

    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < 400:
        return False, 0, 0

    x, y, w, h = cv2.boundingRect(c)
    cx = x + w//2

    # 平滑
    cx = int(alpha*cx + (1-alpha)*last_x)
    area = alpha*area + (1-alpha)*last_area
    last_x, last_area = cx, area

    return True, cx, area

# ================== 主控制 ==================
last_time = time.perf_counter()

def follow(frame):
    global last_time, last_speed

    found, cx, area = detect(frame)

    if not found:
        # 丢失目标 → 慢慢找
        forward(8, 0)
        return frame

    now = time.perf_counter()
    dt = now - last_time
    last_time = now
    if dt > 0.1:
        dt = 0.01

    # ================== 距离 → 速度 ==================
    TARGET = 30000
    error = TARGET - area

    # 连续控制（核心）
    speed = 0.006 * error   # 正→前进 负→后退

    # ================== 死区（防抖核心）==================
    if -300 < error < 300:
        speed = 0

    # ================== 限速 ==================
    speed = np.clip(speed, -30, 30)

    # ================== 速度平滑（非常关键）==================
    speed = 0.7 * last_speed + 0.3 * speed
    last_speed = speed

    # ==================  转向 ==================
    turn = pid_turn.update(cx - CENTER, dt)
    turn *= 0.6

    # ==================  合成 ==================
    if speed >= 0:
        l = speed + turn
        r = speed - turn
        forward(l, r)
    else:
        # 后退转向反向
        l = -speed - turn
        r = -speed + turn
        backward(l, r)

    # ================== UI ==================
    cv2.putText(frame, f"Area:{int(area)}", (10,20),
                cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,255),1)
    cv2.putText(frame, f"Speed:{speed:.1f}", (10,40),
                cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,0),1)

    return frame

# ================== 主循环 ==================
print("=== 视觉跟随启动 ===")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        frame = follow(frame)
        cv2.imshow("Follow", frame)

        if cv2.waitKey(1) == 27:
            break

except KeyboardInterrupt:
    pass

finally:
    stop()
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
