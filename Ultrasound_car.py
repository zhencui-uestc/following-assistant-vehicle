import RPi.GPIO as GPIO
import time
import board
import adafruit_dht

# ================== 初始化 DHT11（使用新库）==================
dht = adafruit_dht.DHT11(board.D17)  # GPIO4
last_valid_temp = 25.0  # 默认温度

# ================== 引脚定义 ==================
PWMA, AIN1, AIN2 = 18, 22, 27
PWMB, BIN1, BIN2 = 23, 25, 24
TRIG, ECHO = 26, 13

# ================== 电机控制 ==================
def up(speed):
    speed = max(0, min(100, speed))
    L_Motor.ChangeDutyCycle(speed)
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN1, True)
    GPIO.output(AIN2, False)
    GPIO.output(BIN1, True)
    GPIO.output(BIN2, False)

def down(speed):
    speed = max(0, min(100, speed))
    L_Motor.ChangeDutyCycle(speed)
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN1, False)
    GPIO.output(AIN2, True)
    GPIO.output(BIN1, False)
    GPIO.output(BIN2, True)

def stop():
    L_Motor.ChangeDutyCycle(0)
    R_Motor.ChangeDutyCycle(0)
    GPIO.output(AIN1, False)
    GPIO.output(AIN2, False)
    GPIO.output(BIN1, False)
    GPIO.output(BIN2, False)

# ================== 温度读取（使用新库）==================
def get_temp():
    global last_valid_temp
    try:
        temperature = dht.temperature
        if temperature is not None:
            last_valid_temp = float(temperature)
    except RuntimeError as e:
        # DHT 常见错误（如读取太快），忽略即可
        pass
    return last_valid_temp

# ================== 距离测量（使用新温度）==================
def distance():
    temp = get_temp()
    speed_sound = (331.3 + 0.606 * temp) * 100  # cm/s

    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG, GPIO.LOW)

    start = time.perf_counter()
    while GPIO.input(ECHO) == 0:
        if time.perf_counter() - start > 0.02:
            return 300

    t1 = time.perf_counter()
    while GPIO.input(ECHO) == 1:
        if time.perf_counter() - t1 > 0.02:
            return 300
    t2 = time.perf_counter()

    dist = (t2 - t1) * speed_sound / 2
    return dist if 2 <= dist <= 300 else 300

# ================== 微摆搜索函数 ==================
def search_target(max_angle=45, turn_speed=35, total_search_time=6.0):
    turn_duration = total_search_time * (max_angle / 180.0)

    # 向右转
    print("搜索中 → 向右扫描...")
    L_Motor.ChangeDutyCycle(turn_speed)
    R_Motor.ChangeDutyCycle(turn_speed)
    GPIO.output(AIN1, True);  GPIO.output(AIN2, False)
    GPIO.output(BIN1, False); GPIO.output(BIN2, True)

    start = time.perf_counter()
    while time.perf_counter() - start < turn_duration:
        if distance() <= 100:
            stop()
            return True
        time.sleep(0.05)

    # 向左转
    print("搜索中 → 向左扫描...")
    L_Motor.ChangeDutyCycle(turn_speed)
    R_Motor.ChangeDutyCycle(turn_speed)
    GPIO.output(AIN1, False); GPIO.output(AIN2, True)
    GPIO.output(BIN1, True);  GPIO.output(BIN2, False)

    start = time.perf_counter()
    while time.perf_counter() - start < 2 * turn_duration:
        if distance() <= 100:
            stop()
            return True
        time.sleep(0.05)

    stop()
    time.sleep(0.1)
    return False

# ================== PID 控制器 ==================
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        self.integral = max(min(self.integral, 10), -10)
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# ================== 系统参数 ==================
TARGET = 10.0
pid = PID(kp=1.6, ki=0.3, kd=0.0)

# ================== 主跟随逻辑 ==================
def follow():
    dist = distance()

    if dist > 100:
        print("目标丢失，启动微摆搜索...")
        if search_target(max_angle=45, turn_speed=35, total_search_time=6.0):
            print(" 重新锁定目标！")
        else:
            print(" 搜索超时，等待目标返回...")
        return

    if dist < 5.0:
        down(20)
        time.sleep(0.7)
        return

    error = dist - TARGET
    now = time.perf_counter()
    if not hasattr(follow, 'last_time'):
        follow.last_time = now
        dt = 0.01
    else:
        dt = now - follow.last_time
        follow.last_time = now

    output = pid.update(error, dt)

    if output < 0:
        down(-output)
    else:
        up(output)

    print(f"跟随中 | 距离:{dist:5.1f}cm | 误差:{error:5.1f} | 输出:{output:5.1f}")

# ================== 初始化 GPIO ==================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

for pin in [AIN1, AIN2, PWMA, BIN1, BIN2, PWMB]:
    GPIO.setup(pin, GPIO.OUT)

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.output(TRIG, GPIO.LOW)

L_Motor = GPIO.PWM(PWMA, 200)
R_Motor = GPIO.PWM(PWMB, 200)
L_Motor.start(0)
R_Motor.start(0)

# ================== 主循环 ==================
print("=== 智能跟随小车启动（带搜索功能）===")
try:
    while True:
        follow()
        time.sleep(0.01)
except KeyboardInterrupt:
    print("\n用户中断")
finally:
    stop()
    L_Motor.stop()
    R_Motor.stop()
    GPIO.cleanup()
    print("已安全退出")
