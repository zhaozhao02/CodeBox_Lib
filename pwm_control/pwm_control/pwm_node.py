import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import Jetson.GPIO as GPIO
import time  # 导入 time 模块

class PWMControlNode(Node):
    def __init__(self):
        super().__init__('pwm_control_node')

        # 初始化GPIO
        self.pwm_pin = 32           # 使用 32 号引脚控制灯
        self.control_pin = 33       # 用于控制高电平输出的引脚
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)     # 禁用警告
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        GPIO.setup(self.control_pin, GPIO.OUT)
        GPIO.output(self.control_pin, GPIO.HIGH)  # 输出高电平
        #     GPIO.output(self.control_pin, GPIO.HIGH)
        #     time.sleep(1)
        #     GPIO.output(self.control_pin, GPIO.LOW)
        #     time.sleep(1)

        # 设置PWM频率和初始占空比
        self.pwm = GPIO.PWM(self.pwm_pin, 50)  # 设置频率为50Hz
        self.pwm.start(0)  # 开始PWM，初始占空比为0

        # 设置最大最小占空比
        self.min_duty = 2.4#2.5  # 对应1000微秒占空比
        self.max_duty = 13.0#12.5  # 对应2000微秒占空比
        self.pwm_duty=8.0
        self.pwm.ChangeDutyCycle(self.pwm_duty)
        # 订阅 PWM 占空比的消息
        self.subscription = self.create_subscription(
            Float32,
            'pwm_duty',
            self.duty_callback,
            10
        )

    def duty_callback(self, msg):
        # 从话题中获取占空比并进行限幅处理
        step=0.2
        duty = msg.data
        if duty == 8:
            self.pwm_duty+=step
        if duty == 7:
            self.pwm_duty-=step
        if duty == 1:
            GPIO.output(self.control_pin, GPIO.HIGH)  # 输出高电平
        if duty == 0:
            GPIO.output(self.control_pin, GPIO.LOW)  # 输出低电平
        if self.pwm_duty < self.min_duty:
            self.pwm_duty+= step
        elif self.pwm_duty > self.max_duty:
            self.pwm_duty-= step
        # 设置 PWM 占空比
        self.pwm.ChangeDutyCycle(self.pwm_duty)
        self.get_logger().info(f"Setting Duty Cycle to: {self.pwm_duty}%")

    def stop_pwm(self):
        self.pwm.ChangeDutyCycle(13.0)
        GPIO.output(self.control_pin, GPIO.LOW)
        time.sleep(1)  # 延迟 1000 毫秒
        self.pwm.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    pwm_control_node = PWMControlNode()

    try:
        rclpy.spin(pwm_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        pwm_control_node.stop_pwm()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
