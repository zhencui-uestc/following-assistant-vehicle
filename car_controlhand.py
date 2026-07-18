import RPi.GPIO as GPIO
import time
from inputs import get_gamepad

# === 电机引脚定义 ===
PWMA = 18
AIN1 = 22
AIN2 = 27

PWMB = 23
BIN1 = 25
BIN2 = 24

# === GPIO 初始化 ===
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup([AIN1, AIN2, BIN1, BIN2], GPIO.OUT)
GPIO.setup([PWMA, PWMB], GPIO.OUT)

L_Motor = GPIO.PWM(PWMA, 100)
R_Motor = GPIO.PWM(PWMB, 100)
L_Motor.start(0)
R_Motor.start(0)

# === 控制参数（适配 0~255 范围）===
MAX_SPEED = 110
MIN_SPEED = 20
current_speed = 60
DEADZONE = 30  # 死区（0~255 范围下合理值）
MIN_MOVE_THRESHOLD = 5  # 最小有效速度

# 手柄摇杆中心值（理想静止位置）
X_CENTER = 128
Y_CENTER = 128


# === 基础运动函数 ===
def go_forward(speed):
    """前进：双轮正转"""
    speed = max(MIN_MOVE_THRESHOLD, min(100, speed))
    L_Motor.ChangeDutyCycle(speed)
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN1, True);
    GPIO.output(AIN2, False)  # 左轮正转
    GPIO.output(BIN1, True);
    GPIO.output(BIN2, False)  # 右轮正转


def go_backward(speed):
    """后退：双轮反转"""
    speed = max(MIN_MOVE_THRESHOLD, min(100, speed))
    L_Motor.ChangeDutyCycle(speed)
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN1, False);
    GPIO.output(AIN2, True)  # 左轮反转
    GPIO.output(BIN1, False);
    GPIO.output(BIN2, True)  # 右轮反转


def turn_left(speed):
    """原地左转：左轮后退，右轮前进"""
    speed = max(MIN_MOVE_THRESHOLD, min(100, speed))
    L_Motor.ChangeDutyCycle(speed)
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN1, False);
    GPIO.output(AIN2, True)  # 左轮反转
    GPIO.output(BIN1, True);
    GPIO.output(BIN2, False)  # 右轮正转


def turn_right(speed):
    """原地右转：左轮前进，右轮后退"""
    speed = max(MIN_MOVE_THRESHOLD, min(100, speed))
    L_Motor.ChangeDutyCycle(speed)
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN1, True);
    GPIO.output(AIN2, False)  # 左轮正转
    GPIO.output(BIN1, False);
    GPIO.output(BIN2, True)  # 右轮反转


def stop():
    """停止所有电机"""
    L_Motor.ChangeDutyCycle(0)
    R_Motor.ChangeDutyCycle(0)
    GPIO.output(AIN1, False)
    GPIO.output(AIN2, False)
    GPIO.output(BIN1, False)
    GPIO.output(BIN2, False)


# === 摇杆控制主逻辑（适配 0~255 输入）===
def drive_with_sticks(lx, ly):
    """
    根据 0~255 范围的摇杆值控制小车
    lx: 左摇杆X (0=左, 255=右)
    ly: 左摇杆Y (0=上, 255=下)
    """
    global current_speed

    # 转换为以中心为0的偏移量（-128 ~ +127）
    x_offset = lx - X_CENTER
    y_offset = ly - Y_CENTER

    # 死区处理
    if abs(x_offset) < DEADZONE:
        x_offset = 0
    if abs(y_offset) < DEADZONE:
        y_offset = 0

    # 判断主方向：前后优先于转向
    if abs(y_offset) > abs(x_offset):
        if y_offset < 0:  # 上推（y < 128）
            go_forward(current_speed)
        elif y_offset > 0:  # 下推（y > 128）
            go_backward(current_speed)
        else:
            stop()
    else:
        if x_offset < 0:  # 左推（x < 128）
            turn_left(current_speed)
        elif x_offset > 0:  # 右推（x > 128）
            turn_right(current_speed)
        else:
            stop()


# === 主循环 ===
print("北通鲲鹏20 手柄控制小车启动！")
print("左摇杆: 控制方向 | A键: 加速 | B键: 减速")
print("输入范围: X=0~255, Y=0~255 (上=0, 下=255, 左=0, 右=255)")

try:
    axes = {'ABS_X': X_CENTER, 'ABS_Y': Y_CENTER}  # 初始居中

    while True:
        events = get_gamepad()
        for event in events:
            if event.ev_type == 'Absolute':
                if event.code in axes:
                    axes[event.code] = event.state
                    drive_with_sticks(axes['ABS_X'], axes['ABS_Y'])

            elif event.ev_type == 'Key':
                if event.code == 'BTN_SOUTH' and event.state == 1:  # A键
                    current_speed = min(current_speed + 10, MAX_SPEED)
                    print(f"加速! 当前速度: {current_speed}")
                    drive_with_sticks(axes['ABS_X'], axes['ABS_Y'])

                elif event.code == 'BTN_EAST' and event.state == 1:  # B键
                    current_speed = max(current_speed - 10, MIN_SPEED)
                    print(f"减速! 当前速度: {current_speed}")
                    drive_with_sticks(axes['ABS_X'], axes['ABS_Y'])

except KeyboardInterrupt:
    print("\n程序被中断")
finally:
    stop()
    L_Motor.stop()
    R_Motor.stop()
    GPIO.cleanup()