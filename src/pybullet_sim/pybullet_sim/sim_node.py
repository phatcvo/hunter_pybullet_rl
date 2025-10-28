#!/usr/bin/env python3
import os
import math
import re
import time
import numpy as np
import pybullet as p
import pybullet_data
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from pybullet_sim.ultis import resolve_package_path


# =====================================================================
# Main Simulation Node
# =====================================================================
class PyBulletHunter(Node):
    def __init__(self):
        super().__init__("pybullet_hunter")

        # ---------------- Parameters ----------------
        self.declare_parameter("use_gui", True)
        self.declare_parameter("update_rate", 100.0)
        self.declare_parameter("max_steer", 0.6)
        self.declare_parameter("wheel_radius", 0.165)
        self.declare_parameter("wheelbase", 0.5)

        self.declare_parameter("static_objects", True)
        self.declare_parameter("dynamic_objects", True)

        self.declare_parameter("publish_lidar", True)
        self.declare_parameter("num_rays", 360)
        self.declare_parameter("lidar_range", 8.0)

        use_gui = bool(self.get_parameter("use_gui").value)
        self.dt = 1.0 / float(self.get_parameter("update_rate").value)
        self.max_steer = float(self.get_parameter("max_steer").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)

        static_objects = bool(self.get_parameter("static_objects").value)
        dynamic_objects = bool(self.get_parameter("dynamic_objects").value)

        self.publish_lidar = bool(self.get_parameter("publish_lidar").value)
        self.num_rays = int(self.get_parameter("num_rays").value)
        self.lidar_range = float(self.get_parameter("lidar_range").value)

        # ---------------- PyBullet setup ----------------
        p.connect(p.GUI if use_gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.dt)
        p.loadURDF("plane.urdf")

        # Disable debug GUI overlays
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

        # ---------------- Load Static Objects ----------------
        if static_objects:
            env_path = os.path.join(
                get_package_share_directory("pybullet_sim"),
                "urdf",
                "static_objects.urdf",
            )
            p.loadURDF(env_path, useFixedBase=True)
            self.get_logger().info(f"Loaded static objects: {env_path}")

        # ---------------- Load Dynamic objects ----------------
        if dynamic_objects:
            obj_path = os.path.join(
                get_package_share_directory("pybullet_sim"),
                "urdf",
                "dynamic_objects.urdf",
            )
            self.dynamic_objs = []
            positions = [[2.0, 2.0, 0.0], [-2.0, 1.5, 0.0], [0.5, -2.5, 0.0]]
            for pos in positions:
                box_id = p.loadURDF(obj_path, pos, useFixedBase=False)
                self.dynamic_objs.append(box_id)
            self.get_logger().info(f"Spawned {len(self.dynamic_objs)} dynamic objects")

        # ---------------- Load Hunter model URDF ----------------
        urdf_path = os.path.join(
            get_package_share_directory("pybullet_sim"), "urdf", "hunter_model.urdf"
        )

        with open(urdf_path, "r") as f:
            urdf_data = f.read()

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
        # Map correct joints
        self.fl_steer = self.joint_names.get("front_left_steering_joint")
        self.fr_steer = self.joint_names.get("front_right_steering_joint")
        self.fl_drive = self.joint_names.get("front_left_wheel_joint")
        self.fr_drive = self.joint_names.get("front_right_wheel_joint")
        self.rl_drive = self.joint_names.get("rear_left_wheel_joint")
        self.rr_drive = self.joint_names.get("rear_right_wheel_joint")

        self.get_logger().info(f"Loaded {self.num_joints} joints:")
        for k, v in self.joint_names.items():
            self.get_logger().info(f"  {v}: {k}")

        self.get_logger().info(
            f"Joint IDs: FLsteer={self.fl_steer}, FRsteer={self.fr_steer}, "
            f"FLdrive={self.fl_drive}, FRdrive={self.fr_drive}, "
            f"RLdrive={self.rl_drive}, RRdrive={self.rr_drive}"
        )
        # Dynamics setup friction for wheels
        for j in [self.fl_steer, self.fr_steer, self.rl_drive, self.rr_drive]:
            if j is not None:
                p.changeDynamics(self.robot, j, lateralFriction=1.0)

        # Command state
        self.v_cmd = 0.0
        self.delta_cmd = 0.0
        # ROS2 I/O
        self.sub = self.create_subscription(
            AckermannDriveStamped, "/cmd", self.cmd_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        if self.publish_lidar:
            self.lidar_pub = self.create_publisher(LaserScan, "/scan", 10)

        # Timer
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("PyBullet Hunter full simulation started.")

    # ------------------------------------------------------------------
    def cmd_callback(self, msg: AckermannDriveStamped):
        self.v_cmd = float(msg.drive.speed)
        self.delta_cmd = max(
            -self.max_steer, min(self.max_steer, float(msg.drive.steering_angle))
        )

    # ------------------------------------------------------------------
    def simulate_lidar(self):
        pos, orn = p.getBasePositionAndOrientation(self.robot)
        yaw = p.getEulerFromQuaternion(orn)[2]
        scan = []
        for i in range(self.num_rays):
            ang = yaw + (i / self.num_rays) * 2 * np.pi
            from_pt = [pos[0], pos[1], pos[2] + 0.2]
            to_pt = [
                pos[0] + self.lidar_range * np.cos(ang),
                pos[1] + self.lidar_range * np.sin(ang),
                pos[2] + 0.2,
            ]
            hit = p.rayTest(from_pt, to_pt)[0]
            dist = (
                np.linalg.norm(np.array(hit[3]) - np.array(from_pt))
                if hit[0] != -1
                else self.lidar_range
            )
            scan.append(dist)

        # Publish LaserScan message
        now = self.get_clock().now().to_msg()
        msg = LaserScan()
        msg.header.stamp = now
        msg.header.frame_id = "base_link"
        msg.angle_min = 0.0
        msg.angle_max = 2 * math.pi
        msg.angle_increment = (2 * math.pi) / self.num_rays
        msg.range_min = 0.05
        msg.range_max = self.lidar_range
        msg.ranges = scan
        self.lidar_pub.publish(msg)

    # ------------------------------------------------------------------
    def update(self):
        # --- Move dynamic objects (optional random oscillation) ---
        t = time.time()
        for i, obj in enumerate(getattr(self, "dynamic_objs", [])):
            x0, y0, _ = p.getBasePositionAndOrientation(obj)[0]
            dx = 0.5 * math.sin(t + i)
            dy = 0.5 * math.cos(t * 0.5 + i)
            p.resetBasePositionAndOrientation(
                obj,
                [x0 + dx * 0.01, y0 + dy * 0.01, 0.0],
                p.getQuaternionFromEuler([0, 0, 0]),
            )

        # --- Steering control ---
        p.setJointMotorControl2(
            self.robot,
            self.fl_steer,
            controlMode=p.POSITION_CONTROL,
            targetPosition=self.delta_cmd,
            force=25.0,
        )
        p.setJointMotorControl2(
            self.robot,
            self.fr_steer,
            controlMode=p.POSITION_CONTROL,
            targetPosition=self.delta_cmd,
            force=25.0,
        )

        # --- Drive wheels ---
        wheel_speed = self.v_cmd / self.wheel_radius
        for j in [self.fl_drive, self.fr_drive, self.rl_drive, self.rr_drive]:
            if j is not None:
                p.setJointMotorControl2(
                    self.robot,
                    j,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=wheel_speed,
                    force=40.0,
                )

        p.stepSimulation()

        # --- Publish odometry ---
        pos, orn = p.getBasePositionAndOrientation(self.robot)
        lin, ang = p.getBaseVelocity(self.robot)
        now = self.get_clock().now()

        # TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = pos[0]
        tf_msg.transform.translation.y = pos[1]
        tf_msg.transform.translation.z = pos[2]
        tf_msg.transform.rotation.x = orn[0]
        tf_msg.transform.rotation.y = orn[1]
        tf_msg.transform.rotation.z = orn[2]
        tf_msg.transform.rotation.w = orn[3]
        self.tf_broadcaster.sendTransform(tf_msg)

        # Odom
        od = Odometry()
        od.header.stamp = now.to_msg()
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = pos[0]
        od.pose.pose.position.y = pos[1]
        od.pose.pose.position.z = pos[2]
        od.pose.pose.orientation.x = orn[0]
        od.pose.pose.orientation.y = orn[1]
        od.pose.pose.orientation.z = orn[2]
        od.pose.pose.orientation.w = orn[3]
        od.twist.twist.linear.x = lin[0]
        od.twist.twist.linear.y = lin[1]
        od.twist.twist.angular.z = ang[2]
        self.odom_pub.publish(od)

        # LiDAR
        if self.publish_lidar:
            self.simulate_lidar()


# =====================================================================
# Main
# =====================================================================
def main():
    rclpy.init()
    node = PyBulletHunter()
    try:
        rclpy.spin(node)
    finally:
        try:
            p.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
