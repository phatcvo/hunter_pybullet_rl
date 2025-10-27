import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import pybullet as p
import pybullet_data
import math
import os
from ament_index_python.packages import get_package_share_directory


def resolve_package_path(path: str) -> str:
    """Convert 'package://pkg_name/...' to absolute filesystem path."""
    if not path.startswith("package://"):
        return path  # already normal path
    pkg_name = path.split("/")[2]
    rel_path = "/".join(path.split("/")[3:])
    pkg_share = get_package_share_directory(pkg_name)
    return os.path.join(pkg_share, rel_path)


class PyBulletHunter(Node):
    def __init__(self):
        super().__init__("hunter_sim")

        # Parameters
        self.declare_parameter("use_gui", True)
        self.declare_parameter("update_rate", 100.0)
        self.declare_parameter("max_steer", 0.6)
        self.declare_parameter("wheel_radius", 0.165)  # Hunter wheel radius
        self.declare_parameter("wheelbase", 0.5)  # front–rear distance

        use_gui = bool(self.get_parameter("use_gui").value)
        self.dt = 1.0 / float(self.get_parameter("update_rate").value)
        self.max_steer = float(self.get_parameter("max_steer").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)

        # PyBullet setup
        p.connect(p.GUI if use_gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.dt)

        p.loadURDF("plane.urdf")

        urdf_path = os.path.join(
            get_package_share_directory("pybullet_sim"), "urdf", "hunter_model.urdf"
        )

        # Read file and replace 'package://' entries with real absolute paths
        with open(urdf_path, "r") as f:
            urdf_data = f.read()

        import re

        matches = re.findall(r'package://[^\s"]+', urdf_data)
        for match in matches:
            abs_path = resolve_package_path(match)
            urdf_data = urdf_data.replace(match, f"file://{abs_path}")

        tmp_urdf = "/tmp/hunter_model_resolved.urdf"
        with open(tmp_urdf, "w") as f:
            f.write(urdf_data)

        start_pos = [0.0, 0.0, 0.2]
        start_ori = p.getQuaternionFromEuler([0, 0, 0])
        self.robot = p.loadURDF(tmp_urdf, start_pos, start_ori)

        # Get joint names from URDF
        self.num_joints = p.getNumJoints(self.robot)
        self.joint_names = {
            p.getJointInfo(self.robot, i)[1].decode(): i for i in range(self.num_joints)
        }

        self.get_logger().info(f"Loaded {self.num_joints} joints from Hunter URDF:")
        for k, v in self.joint_names.items():
            self.get_logger().info(f"  {v}: {k}")

        # Map correct joints
        self.fl_steer = self.joint_names.get("front_left_steering_joint")
        self.fr_steer = self.joint_names.get("front_right_steering_joint")
        self.fl_drive = self.joint_names.get("front_left_wheel_joint")
        self.fr_drive = self.joint_names.get("front_right_wheel_joint")
        self.rl_drive = self.joint_names.get("rear_left_wheel_joint")
        self.rr_drive = self.joint_names.get("rear_right_wheel_joint")

        self.get_logger().info(
            f"Joint IDs: FLsteer={self.fl_steer}, FRsteer={self.fr_steer}, "
            f"FLdrive={self.fl_drive}, FRdrive={self.fr_drive}, "
            f"RLdrive={self.rl_drive}, RRdrive={self.rr_drive}"
        )
        # Dynamics setup
        for j in [self.fl_steer, self.fr_steer, self.rl_drive, self.rr_drive]:
            if j is not None:
                p.changeDynamics(self.robot, j, lateralFriction=1.2)

        # Command state
        self.v_cmd = 0.0
        self.delta_cmd = 0.0

        # ROS I/O
        self.sub = self.create_subscription(
            AckermannDriveStamped, "/cmd", self.cmd_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("PyBullet Hunter simulation started.")

    # ------------------------------------------------------------------
    def cmd_callback(self, msg: AckermannDriveStamped):
        self.v_cmd = float(msg.drive.speed)
        self.delta_cmd = max(
            -self.max_steer, min(self.max_steer, float(msg.drive.steering_angle))
        )

    # ------------------------------------------------------------------
    def update(self):
        # Apply steering control
        steer_angle = self.delta_cmd

        p.setJointMotorControl2(
            self.robot,
            self.fl_steer,
            controlMode=p.POSITION_CONTROL,
            targetPosition=steer_angle,
            force=3.0,
        )
        p.setJointMotorControl2(
            self.robot,
            self.fr_steer,
            controlMode=p.POSITION_CONTROL,
            targetPosition=steer_angle,
            force=3.0,
        )

        # Drive wheels
        wheel_speed = self.v_cmd / self.wheel_radius
        # --- Steering (front) ---
        for j in [self.fl_steer, self.fr_steer]:
            p.setJointMotorControl2(
                self.robot,
                j,
                controlMode=p.POSITION_CONTROL,
                targetPosition=self.delta_cmd,
                force=5.0,
            )

        # --- Drive (4 wheels) ---
        for j in [self.fl_drive, self.fr_drive, self.rl_drive, self.rr_drive]:
            if j is not None:
                p.setJointMotorControl2(
                    self.robot,
                    j,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=wheel_speed,
                    force=20.0,
                )

        p.stepSimulation()

        # Get robot state
        pos, orn = p.getBasePositionAndOrientation(self.robot)
        lin, ang = p.getBaseVelocity(self.robot)

        x, y, z = pos
        vx, vy, wz = lin[0], lin[1], ang[2]

        now = self.get_clock().now()

        # Publish TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = float(x)
        tf_msg.transform.translation.y = float(y)
        tf_msg.transform.translation.z = float(z)
        tf_msg.transform.rotation.x = float(orn[0])
        tf_msg.transform.rotation.y = float(orn[1])
        tf_msg.transform.rotation.z = float(orn[2])
        tf_msg.transform.rotation.w = float(orn[3])
        self.tf_broadcaster.sendTransform(tf_msg)

        # Publish Odometry
        od = Odometry()
        od.header.stamp = now.to_msg()
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = float(x)
        od.pose.pose.position.y = float(y)
        od.pose.pose.position.z = float(z)
        od.pose.pose.orientation.x = float(orn[0])
        od.pose.pose.orientation.y = float(orn[1])
        od.pose.pose.orientation.z = float(orn[2])
        od.pose.pose.orientation.w = float(orn[3])
        od.twist.twist.linear.x = float(vx)
        od.twist.twist.linear.y = float(vy)
        od.twist.twist.angular.z = float(wz)
        self.odom_pub.publish(od)

    # ------------------------------------------------------------------


def main():
    rclpy.init()
    node = PyBulletHunter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        p.disconnect()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
