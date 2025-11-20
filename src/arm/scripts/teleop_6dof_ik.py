#!/usr/bin/env python3
"""
6-DOF Analytical IK Teleop - Full position and orientation control
Uses only Python standard library (math module) - no external dependencies!

This implements a 6-DOF IK solver for the arm:
- Base rotation (yaw)
- Shoulder + Elbow + Wrist (3-link arm for position + orientation)
- Gripper rotation

Controls:
  Position (meters):
    w/s : +X / -X (forward/back)
    a/d : +Y / -Y (left/right)
    r/f : +Z / -Z (up/down)

  Orientation (radians):
    i/k : +pitch / -pitch (tilt up/down)
    j/l : +roll / -roll (rotate left/right)
    u/o : +yaw / -yaw (twist CCW/CW)

  Steps:
    [ / ] : decrease / increase step (pos: 0.01m, ori: 0.1rad)

  Presets:
    0 : reset to default pose
    h : print help
    q : quit
"""

import math
import sys
import tty
import termios
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped


# Arm parameters from URDF (in meters)
L0 = 0.2104  # Base to shoulder (z-offset)
L1 = 0.4186  # Shoulder to elbow  
L2 = 0.4159  # Elbow to wrist
L3 = 0.1591  # Wrist to gripper (0.1025 + 0.05655)


def solve_6dof_ik(x, y, z, roll, pitch, yaw):
    """
    Solve 6-DOF inverse kinematics for the arm.
    
    Args:
        x, y, z: Target position (meters)
        roll, pitch, yaw: Target orientation (radians)
    
    Returns:
        (base, shoulder, elbow, wrist, gripper) joint angles in radians
        or None if unreachable
    """
    
    # 1. Base angle (yaw component)
    base_angle = math.atan2(y, x)
    
    # 2. Calculate wrist center position (subtract tool offset along approach vector)
    # The approach vector is determined by pitch and yaw
    wrist_x = x - L3 * math.cos(pitch) * math.cos(yaw - base_angle)
    wrist_y = y - L3 * math.cos(pitch) * math.sin(yaw - base_angle)
    wrist_z = z - L3 * math.sin(pitch)
    
    # 3. Convert to 2D problem in vertical plane through base
    r_horiz = math.sqrt(wrist_x**2 + wrist_y**2)
    r_vert = wrist_z - L0  # Subtract base height
    r = math.sqrt(r_horiz**2 + r_vert**2)
    
    # 4. Check reachability
    if r > (L1 + L2) or r < abs(L1 - L2):
        return None
    
    # 5. Solve 2-link planar IK for shoulder and elbow
    # Law of cosines for elbow angle
    cos_elbow = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    
    # Clamp to valid range to avoid math domain errors
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    
    # Elbow angle (negative for elbow-down configuration)
    elbow_angle = -math.acos(cos_elbow)
    
    # Shoulder angle
    alpha = math.atan2(r_vert, r_horiz)
    beta = math.atan2(L2 * math.sin(abs(elbow_angle)), L1 + L2 * math.cos(elbow_angle))
    shoulder_angle = alpha - beta
    
    # 6. Wrist angle to achieve desired pitch
    # The wrist needs to compensate for shoulder and elbow angles
    wrist_angle = pitch - shoulder_angle - elbow_angle
    
    # 7. Gripper angle for roll/yaw compensation
    # Simple approximation: use roll for gripper orientation
    gripper_angle = roll
    
    return (base_angle, shoulder_angle, elbow_angle, wrist_angle, gripper_angle)


class TeleopIKNode(Node):
    def __init__(self):
        super().__init__('teleop_6dof_ik')
        
        # Joint publishers
        self.pub_base = self.create_publisher(Float64, '/arm/arm_base_joint/cmd_pos', 10)
        self.pub_shoulder = self.create_publisher(Float64, '/arm/arm_shoulder_joint/cmd_pos', 10)
        self.pub_elbow = self.create_publisher(Float64, '/arm/arm_elbow_joint/cmd_pos', 10)
        self.pub_wrist = self.create_publisher(Float64, '/arm/arm_wrist_joint/cmd_pos', 10)
        self.pub_gripper = self.create_publisher(Float64, '/arm/arm_gripper_base_joint/cmd_pos', 10)
        
        # Target pose publisher (for visualization)
        self.pub_target = self.create_publisher(PoseStamped, '/arm/target_pose', 10)
        
        # Current target pose
        self.target_x = 0.4
        self.target_y = 0.0
        self.target_z = 0.6
        self.target_roll = 0.0
        self.target_pitch = -math.pi/4  # Point slightly downward
        self.target_yaw = 0.0
        
        # Step sizes
        self.pos_step = 0.01  # meters
        self.ori_step = 0.1   # radians
        
        # Joint limits (from URDF)
        self.limits = {
            'base': (-3.14, 3.14),
            'shoulder': (-1.918, 1.918),
            'elbow': (-1.748, 1.748),
            'wrist': (-1.942, 1.942),
            'gripper': (-1.57, 1.57)
        }
        
        self.get_logger().info('6-DOF IK Teleop started!')
        self.get_logger().info('Press h for help, q to quit')
        
        # Send initial pose
        self.update_target()
    
    def clamp_joints(self, angles):
        """Clamp joint angles to their limits."""
        if angles is None:
            return None
        
        base, shoulder, elbow, wrist, gripper = angles
        
        base = max(self.limits['base'][0], min(self.limits['base'][1], base))
        shoulder = max(self.limits['shoulder'][0], min(self.limits['shoulder'][1], shoulder))
        elbow = max(self.limits['elbow'][0], min(self.limits['elbow'][1], elbow))
        wrist = max(self.limits['wrist'][0], min(self.limits['wrist'][1], wrist))
        gripper = max(self.limits['gripper'][0], min(self.limits['gripper'][1], gripper))
        
        return (base, shoulder, elbow, wrist, gripper)
    
    def update_target(self):
        """Solve IK and publish joint commands."""
        # Solve IK
        angles = solve_6dof_ik(
            self.target_x, self.target_y, self.target_z,
            self.target_roll, self.target_pitch, self.target_yaw
        )
        
        if angles is None:
            self.get_logger().warn(f'Target unreachable: ({self.target_x:.3f}, {self.target_y:.3f}, {self.target_z:.3f})')
            return False
        
        # Clamp to joint limits
        angles = self.clamp_joints(angles)
        if angles is None:
            return False
        
        base, shoulder, elbow, wrist, gripper = angles
        
        # Publish commands
        msg = Float64()
        
        msg.data = base
        self.pub_base.publish(msg)
        
        msg.data = shoulder
        self.pub_shoulder.publish(msg)
        
        msg.data = elbow
        self.pub_elbow.publish(msg)
        
        msg.data = wrist
        self.pub_wrist.publish(msg)
        
        msg.data = gripper
        self.pub_gripper.publish(msg)
        
        # Publish target pose for visualization
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.pose.position.x = self.target_x
        pose_msg.pose.position.y = self.target_y
        pose_msg.pose.position.z = self.target_z
        
        # Convert RPY to quaternion (simplified)
        cy = math.cos(self.target_yaw * 0.5)
        sy = math.sin(self.target_yaw * 0.5)
        cp = math.cos(self.target_pitch * 0.5)
        sp = math.sin(self.target_pitch * 0.5)
        cr = math.cos(self.target_roll * 0.5)
        sr = math.sin(self.target_roll * 0.5)
        
        pose_msg.pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose_msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose_msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose_msg.pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        self.pub_target.publish(pose_msg)
        
        self.get_logger().info(
            f'Target: ({self.target_x:.3f}, {self.target_y:.3f}, {self.target_z:.3f}) '
            f'RPY: ({math.degrees(self.target_roll):.1f}°, {math.degrees(self.target_pitch):.1f}°, {math.degrees(self.target_yaw):.1f}°) | '
            f'Joints: [{math.degrees(base):.1f}°, {math.degrees(shoulder):.1f}°, {math.degrees(elbow):.1f}°, {math.degrees(wrist):.1f}°, {math.degrees(gripper):.1f}°]'
        )
        
        return True
    
    def reset_pose(self):
        """Reset to default pose."""
        self.target_x = 0.4
        self.target_y = 0.0
        self.target_z = 0.6
        self.target_roll = 0.0
        self.target_pitch = -math.pi/4
        self.target_yaw = 0.0
        self.update_target()
    
    def print_help(self):
        """Print help message."""
        print("\n" + "="*60)
        print("6-DOF IK Teleop Controls")
        print("="*60)
        print("\nPOSITION CONTROL:")
        print("  w/s : Move forward/backward (+X/-X)")
        print("  a/d : Move left/right (+Y/-Y)")
        print("  r/f : Move up/down (+Z/-Z)")
        print("\nORIENTATION CONTROL:")
        print("  i/k : Pitch up/down (tilt end-effector)")
        print("  j/l : Roll left/right (rotate end-effector)")
        print("  u/o : Yaw CCW/CW (twist end-effector)")
        print("\nSTEP SIZE:")
        print(f"  [   : Decrease step (pos: {self.pos_step*0.5:.4f}m, ori: {self.ori_step*0.5:.2f}rad)")
        print(f"  ]   : Increase step (pos: {self.pos_step*2:.4f}m, ori: {self.ori_step*2:.2f}rad)")
        print(f"  Current: pos={self.pos_step:.4f}m, ori={self.ori_step:.2f}rad")
        print("\nOTHER:")
        print("  0   : Reset to default pose")
        print("  h   : Show this help")
        print("  q   : Quit")
        print("="*60 + "\n")


def get_key(settings):
    """Get a single keypress."""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init(args=args)
    node = TeleopIKNode()
    
    # Print initial help
    node.print_help()
    
    try:
        while rclpy.ok():
            key = get_key(settings)
            
            if key == 'q':
                break
            elif key == 'h':
                node.print_help()
            elif key == '0':
                node.reset_pose()
            
            # Position control
            elif key == 'w':
                node.target_x += node.pos_step
                node.update_target()
            elif key == 's':
                node.target_x -= node.pos_step
                node.update_target()
            elif key == 'a':
                node.target_y += node.pos_step
                node.update_target()
            elif key == 'd':
                node.target_y -= node.pos_step
                node.update_target()
            elif key == 'r':
                node.target_z += node.pos_step
                node.update_target()
            elif key == 'f':
                node.target_z -= node.pos_step
                node.update_target()
            
            # Orientation control
            elif key == 'i':
                node.target_pitch += node.ori_step
                node.update_target()
            elif key == 'k':
                node.target_pitch -= node.ori_step
                node.update_target()
            elif key == 'j':
                node.target_roll += node.ori_step
                node.update_target()
            elif key == 'l':
                node.target_roll -= node.ori_step
                node.update_target()
            elif key == 'u':
                node.target_yaw += node.ori_step
                node.update_target()
            elif key == 'o':
                node.target_yaw -= node.ori_step
                node.update_target()
            
            # Step size control
            elif key == '[':
                node.pos_step = max(0.001, node.pos_step * 0.5)
                node.ori_step = max(0.01, node.ori_step * 0.5)
                node.get_logger().info(f'Step size: {node.pos_step:.4f}m, {node.ori_step:.2f}rad')
            elif key == ']':
                node.pos_step = min(0.1, node.pos_step * 2.0)
                node.ori_step = min(0.5, node.ori_step * 2.0)
                node.get_logger().info(f'Step size: {node.pos_step:.4f}m, {node.ori_step:.2f}rad')
            
            # Escape sequences (arrow keys, etc.)
            elif key == '\x1b':
                # Consume the rest of the escape sequence
                sys.stdin.read(2)
    
    except KeyboardInterrupt:
        pass
    
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print('\nTeleop stopped.')


if __name__ == '__main__':
    main()
