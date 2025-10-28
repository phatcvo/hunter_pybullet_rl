import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import sys, termios, tty


class KeyboardDrive(Node):
    def __init__(self):
        super().__init__("keyboard_drive")
        self.pub = self.create_publisher(AckermannDriveStamped, "/cmd", 10)
        self.speed = 0.0
        self.steer = 0.0
        self.get_logger().info("Use W/S for speed, A/D for steering, Q to quit")

    def run(self):
        old = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while True:
                key = sys.stdin.read(1)
                if key == "q":
                    break
                elif key == "w":
                    self.speed += 0.1
                elif key == "s":
                    self.speed -= 0.1
                elif key == "a":
                    self.steer += 0.05
                elif key == "d":
                    self.steer -= 0.05

                msg = AckermannDriveStamped()
                msg.drive.speed = float(self.speed)
                msg.drive.steering_angle = float(self.steer)
                self.pub.publish(msg)
                self.get_logger().info(
                    f"Speed={self.speed:.2f}, Steer={self.steer:.2f}"
                )
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = KeyboardDrive()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
