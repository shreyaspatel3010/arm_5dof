
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import numpy as np
from ik_solver import CHAIN_JOINTS, ik_solve_se3  # uses 6D solver if present; else falls back to pos-only

# Map joint name -> ROS topic used by your Gazebo bridge
BRIDGE_TOPICS = {
    "arm_base_joint": "/arm/arm_base_joint/cmd_pos",
    "arm_shoulder_joint": "/arm/arm_shoulder_joint/cmd_pos",
    "arm_elbow_joint": "/arm/arm_elbow_joint/cmd_pos",
    "arm_wrist_joint": "/arm/arm_wrist_joint/cmd_pos",
    "arm_gripper_base_joint": "/arm/arm_gripper_base_joint/cmd_pos",
    # Fingers exist too, but are not driven by IK:
    # "arm_gripper_left_joint": "/arm/arm_gripper_left_joint/cmd_pos",
    # "arm_gripper_right_joint": "/arm/arm_gripper_right_joint/cmd_pos",
}

class IKBridgeNode(Node):
    def __init__(self):
        super().__init__('ik_bridge_node')
        self.sub = self.create_subscription(PoseStamped, 'ee_target', self.cb, 10)
        self.pubs = {name: self.create_publisher(Float64, topic, 10) for name, topic in BRIDGE_TOPICS.items()}
        self.q = np.zeros(len(CHAIN_JOINTS))
        self.get_logger().info(f"IK Bridge ready. Driving topics via Gazebo bridge. Joints: {CHAIN_JOINTS}")
        self._last_log = self.get_clock().now()

    def cb(self, msg: PoseStamped):
        t = msg.pose.position
        q = msg.pose.orientation
        target_pos = np.array([t.x, t.y, t.z])
        target_quat = np.array([q.x, q.y, q.z, q.w])

        # Use 6D solver if available; otherwise ik_solve_se3 should resolve gracefully
        q_sol, p_sol, _ = ik_solve_se3(target_pos, target_quat, q0=self.q)
        self.q = q_sol

        # Publish each joint angle to its Float64 topic
        for name, angle in zip(CHAIN_JOINTS, q_sol.tolist()):
            if name in self.pubs:
                msg = Float64()
                msg.data = float(angle)
                self.pubs[name].publish(msg)

        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds > 5e8:
            pos_err = float(np.linalg.norm(target_pos - p_sol))
            self.get_logger().info(f"cmd sent. pos_err={pos_err:.4f} m")
            self._last_log = now

def main():
    rclpy.init()
    node = IKBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
