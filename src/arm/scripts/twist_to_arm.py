#!/usr/bin/env python3
import json
from typing import Dict, Tuple
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# Default mapping from Twist fields to (joint_name, gain_rad_per_sec)
DEFAULT_MAP: Dict[str, Tuple[str, float]] = {
    "angular.z": ("arm_base_joint",  0.8),
    "linear.x":  ("arm_shoulder_joint", 0.8),
    "linear.y":  ("arm_elbow_joint",   0.8),
    "angular.x": ("arm_wrist_joint",   0.8),
    "angular.y": ("arm_gripper_base_joint", 0.8),
}

# Joint limits (rad) — tune to match your URDF
JOINT_LIMITS = {
    "arm_base_joint": (-3.14, 3.14),
    "arm_shoulder_joint": (-1.57, 1.57),
    "arm_elbow_joint": (-1.57, 1.57),
    "arm_wrist_joint": (-1.57, 1.57),
    "arm_gripper_base_joint": (-1.57, 1.57),
}

BRIDGE_TOPIC_FMT = "/arm/{joint}/cmd_pos"
GRIPPER_LEFT_TOPIC = "/arm/arm_gripper_left_joint/cmd_pos"
GRIPPER_RIGHT_TOPIC = "/arm/arm_gripper_right_joint/cmd_pos"

class TwistToArm(Node):
    def __init__(self):
        super().__init__("twist_to_arm")
        # Simple parameter types only (ROS 2): strings, numbers, bools, arrays thereof.
        self.declare_parameter("input_twist", "/cmd_vel")
        self.declare_parameter("rate", 60.0)  # Hz
        # Provide mapping override via JSON string to remain type-safe
        self.declare_parameter("mapping_json", "")
        self.declare_parameter("gripper_open", 0.0)
        self.declare_parameter("gripper_closed", 0.08925)

        self.input_topic = self.get_parameter("input_twist").get_parameter_value().string_value
        self.rate = float(self.get_parameter("rate").value)
        self.dt = 1.0 / max(self.rate, 1.0)

        # Parse mapping from JSON string if provided, else DEFAULT_MAP
        mapping_json = self.get_parameter("mapping_json").get_parameter_value().string_value
        self.mapping: Dict[str, Tuple[str, float]] = DEFAULT_MAP
        if mapping_json:
            try:
                parsed = json.loads(mapping_json)
                # Expect {"angular.z": ["arm_base_joint", 0.8], ...}
                m: Dict[str, Tuple[str, float]] = {}
                for k, v in parsed.items():
                    m[k] = (str(v[0]), float(v[1]))
                self.mapping = m
                self.get_logger().info("Loaded mapping from mapping_json parameter.")
            except Exception as e:
                self.get_logger().warn(f"Failed to parse mapping_json; using defaults. Error: {e}")

        # Initialize joint setpoints at 0
        self.joint_pos: Dict[str, float] = {jn: 0.0 for jn in JOINT_LIMITS.keys()}
        self.gripper_pos = float(self.get_parameter("gripper_closed").value)

        # Publishers
        self.pubs: Dict[str, any] = {}
        for jn in self.joint_pos.keys():
            topic = BRIDGE_TOPIC_FMT.format(joint=jn)
            self.pubs[jn] = self.create_publisher(Float64, topic, 10)
            self.get_logger().info(f"Publishing {jn} to {topic}")

        self.pub_left = self.create_publisher(Float64, GRIPPER_LEFT_TOPIC, 10)
        self.pub_right = self.create_publisher(Float64, GRIPPER_RIGHT_TOPIC, 10)

        self.sub = self.create_subscription(Twist, self.input_topic, self.on_twist, 10)

        self.last_twist = Twist()
        self.timer = self.create_timer(self.dt, self.on_timer)

    def on_twist(self, msg: Twist):
        self.last_twist = msg

    def on_timer(self):
        # integrate velocities from last_twist into joint positions
        for field, (joint, gain) in self.mapping.items():
            v = self._get_field(self.last_twist, field)
            dq = float(gain) * float(v) * self.dt
            lo, hi = JOINT_LIMITS[joint]
            x = self.joint_pos[joint] + dq
            if x < lo: x = lo
            if x > hi: x = hi
            self.joint_pos[joint] = x

        # Publish joints
        out = Float64()
        for jn, pub in self.pubs.items():
            out.data = self.joint_pos[jn]
            pub.publish(out)

        # Gripper: map linear.z to open/close (simple proxy)
        gz = self.last_twist.linear.z
        open_pos = float(self.get_parameter("gripper_open").value)
        closed_pos = float(self.get_parameter("gripper_closed").value)
        target = open_pos if gz > 0.2 else (closed_pos if gz < -0.2 else self.gripper_pos)
        self.gripper_pos += (target - self.gripper_pos) * 0.2
        m = Float64()
        m.data = self.gripper_pos
        self.pub_left.publish(m)
        self.pub_right.publish(m)

    @staticmethod
    def _get_field(msg: Twist, dotted: str) -> float:
        obj = msg
        for part in dotted.split("."):
            obj = getattr(obj, part)
        return float(obj)

def main():
    rclpy.init()
    node = TwistToArm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
