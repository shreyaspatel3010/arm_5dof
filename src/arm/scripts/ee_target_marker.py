#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer


FRAME_ID = "base_link"
MARKER_NAME = "ee_target"
MARKER_SCALE = 0.20

def make_axis_marker(scale=0.2):
    def arrow(axis):
        m = Marker()
        m.type = Marker.ARROW
        m.scale.x = scale * 0.7
        m.scale.y = scale * 0.04
        m.scale.z = scale * 0.06
        if axis == 'x':
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.2, 0.9
            m.pose.orientation.w = 1.0
        elif axis == 'y':
            m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 1.0, 0.2, 0.9
            m.pose.orientation.z = 0.7071
            m.pose.orientation.w = 0.7071
        else:
            m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.6, 1.0, 0.9
            m.pose.orientation.y = -0.7071
            m.pose.orientation.w = 0.7071
        return m
    return [arrow('x'), arrow('y'), arrow('z')]

def add_6dof_controls(im: InteractiveMarker):
    for axis, ori in [('x',(1.0,0.0,0.0,1.0)),
                      ('y',(0.0,1.0,0.0,1.0)),
                      ('z',(0.0,0.0,1.0,1.0))]:
        ctrl = InteractiveMarkerControl()
        ctrl.orientation.x = float(ori[0]); ctrl.orientation.y = float(ori[1])
        ctrl.orientation.z = float(ori[2]); ctrl.orientation.w = float(ori[3])
        ctrl.name = f"rotate_{axis}"
        ctrl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        im.controls.append(ctrl)

        ctrl = InteractiveMarkerControl()
        ctrl.orientation.x = float(ori[0]); ctrl.orientation.y = float(ori[1])
        ctrl.orientation.z = float(ori[2]); ctrl.orientation.w = float(ori[3])
        ctrl.name = f"move_{axis}"
        ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        im.controls.append(ctrl)

def normalize_quat(x,y,z,w):
    n = math.sqrt(float(x)*float(x)+float(y)*float(y)+float(z)*float(z)+float(w)*float(w))
    if n < 1e-12: return 0.0,0.0,0.0,1.0
    return float(x)/n, float(y)/n, float(z)/n, float(w)/n

class EETargetMarker(Node):
    def __init__(self):
        super().__init__("ee_target_marker")
        self.pub = self.create_publisher(PoseStamped, "ee_target", 10)
        self.server = InteractiveMarkerServer(self, "ee_target_server")
        self.create_marker()
        self.get_logger().info("Interactive marker ready: drag/rotate in RViz to set /ee_target")

    def create_marker(self):
        im = InteractiveMarker()
        im.header.frame_id = FRAME_ID
        im.name = MARKER_NAME
        im.description = "EE Target"
        im.scale = float(MARKER_SCALE)

        ctrl = InteractiveMarkerControl()
        ctrl.always_visible = True
        for m in make_axis_marker(scale=MARKER_SCALE):
            ctrl.markers.append(m)
        im.controls.append(ctrl)

        add_6dof_controls(im)

        im.pose.position.x = 0.25
        im.pose.position.y = 0.0
        im.pose.position.z = 0.90
        im.pose.orientation.x = 0.0
        im.pose.orientation.y = 0.0
        im.pose.orientation.z = 0.0
        im.pose.orientation.w = 1.0

        self.server.insert(im)
        self.server.setCallback(MARKER_NAME, self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, fb):
        px,py,pz = float(fb.pose.position.x), float(fb.pose.position.y), float(fb.pose.position.z)
        qx,qy,qz,qw = normalize_quat(fb.pose.orientation.x, fb.pose.orientation.y,
                                     fb.pose.orientation.z, fb.pose.orientation.w)
        ps = PoseStamped()
        ps.header.frame_id = fb.header.frame_id or FRAME_ID
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = px; ps.pose.position.y = py; ps.pose.position.z = pz
        ps.pose.orientation.x = qx; ps.pose.orientation.y = qy; ps.pose.orientation.z = qz; ps.pose.orientation.w = qw
        self.pub.publish(ps)

def main():
    rclpy.init()
    node = EETargetMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
