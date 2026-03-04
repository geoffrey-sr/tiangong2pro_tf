#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from bodyctrl_msgs.msg import MotorStatusMsg
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET
import os
import math
import time
import numpy as np

def euler_to_quaternion(r, p, y):
    # Standard conversion
    qx = np.sin(r/2) * np.cos(p/2) * np.cos(y/2) - np.cos(r/2) * np.sin(p/2) * np.sin(y/2)
    qy = np.cos(r/2) * np.sin(p/2) * np.cos(y/2) + np.sin(r/2) * np.cos(p/2) * np.sin(y/2)
    qz = np.cos(r/2) * np.cos(p/2) * np.sin(y/2) - np.sin(r/2) * np.sin(p/2) * np.cos(y/2)
    qw = np.cos(r/2) * np.cos(p/2) * np.cos(y/2) + np.sin(r/2) * np.sin(p/2) * np.sin(y/2)
    return qx, qy, qz, qw

# Mapping from ID to Joint Name
MOTOR_ID_TO_JOINT = {
    1: 'head_roll_joint',
    2: 'head_pitch_joint',
    3: 'head_yaw_joint',
    
    11: 'shoulder_pitch_l_joint',
    12: 'shoulder_roll_l_joint',
    13: 'shoulder_yaw_l_joint',
    14: 'elbow_pitch_l_joint',
    15: 'elbow_yaw_l_joint',
    16: 'wrist_pitch_l_joint',
    17: 'wrist_roll_l_joint',
    
    21: 'shoulder_pitch_r_joint',
    22: 'shoulder_roll_r_joint',
    23: 'shoulder_yaw_r_joint',
    24: 'elbow_pitch_r_joint',
    25: 'elbow_yaw_r_joint',
    26: 'wrist_pitch_r_joint',
    27: 'wrist_roll_r_joint',
    
    31: 'body_yaw_joint',
    
    51: 'hip_roll_l_joint',
    52: 'hip_pitch_l_joint',
    53: 'hip_yaw_l_joint',
    54: 'knee_pitch_l_joint',
    55: 'ankle_pitch_l_joint',
    56: 'ankle_roll_l_joint',
    
    61: 'hip_roll_r_joint',
    62: 'hip_pitch_r_joint',
    63: 'hip_yaw_r_joint',
    64: 'knee_pitch_r_joint',
    65: 'ankle_pitch_r_joint',
    66: 'ankle_roll_r_joint'
}

# Hand limits (Upper limits, Lower is 0)
# ID 1: Little, 2: Ring, 3: Middle, 4: Index, 5: Thumb Bend, 6: Thumb Rot
LEFT_HAND_JOINTS = {
    1: ('left_little_1_joint', 1.333),
    2: ('left_ring_1_joint', 1.333),
    3: ('left_middle_1_joint', 1.333),
    4: ('left_index_1_joint', 1.333),
    5: ('left_thumb_2_joint', 0.48),
    6: ('left_thumb_1_joint', 1.246165)
}

RIGHT_HAND_JOINTS = {
    1: ('right_little_1_joint', 1.333),
    2: ('right_ring_1_joint', 1.333),
    3: ('right_middle_1_joint', 1.333),
    4: ('right_index_1_joint', 1.333),
    5: ('right_thumb_2_joint', 0.48),
    6: ('right_thumb_1_joint', 1.246165)
}

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('tg_joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.status_pub_ = self.create_publisher(Bool, '/robot_online_status', 10)
        self.marker_pub_ = self.create_publisher(Marker, '/robot_status_marker', 10)
        self.offline_visual_pub_ = self.create_publisher(MarkerArray, '/offline_robot_markers', 10)
        
        self.visuals = {} # link_name -> list of (mesh_path, xyz, rpy)
        self._load_urdf_visuals()

        # Subscriptions
        # Correct message type based on description: MotorStatusMsg.msg contains MotorStatus[] status
        # And the topic description says "数据定义位置：bodyctrl_msgs::msg::MotorStatusMsg.msg"
        self.create_subscription(MotorStatusMsg, '/head/status', self.motor_status_callback, 10)
        self.create_subscription(MotorStatusMsg, '/waist/status', self.motor_status_callback, 10)
        self.create_subscription(MotorStatusMsg, '/arm/status', self.motor_status_callback, 10)
        self.create_subscription(MotorStatusMsg, '/leg/status', self.motor_status_callback, 10)
        
        self.create_subscription(JointState, '/inspire_hand/state/left_hand', self.left_hand_callback, 10)
        self.create_subscription(JointState, '/inspire_hand/state/right_hand', self.right_hand_callback, 10)
        
        self.joint_positions = {}
        
        # Initialize all joints to 0.0 to avoid warnings when no robot is connected
        for joint_name in MOTOR_ID_TO_JOINT.values():
            self.joint_positions[joint_name] = 0.0

        for joint_name, _ in LEFT_HAND_JOINTS.values():
            self.joint_positions[joint_name] = 0.0

        for joint_name, _ in RIGHT_HAND_JOINTS.values():
            self.joint_positions[joint_name] = 0.0
            
        self.last_data_time = 0.0
        self.timer = self.create_timer(0.02, self.publish_joint_states) # 50Hz

    def _load_urdf_visuals(self):
        try:
            pkg_path = get_package_share_directory('tiangong2pro_urdf')
            urdf_path = os.path.join(pkg_path, 'urdf', 'tiangong2.0_pro_with_hands.urdf')
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            for link in root.findall('link'):
                link_name = link.get('name')
                self.visuals[link_name] = []
                for visual in link.findall('visual'):
                    geometry = visual.find('geometry')
                    if geometry is not None:
                        mesh = geometry.find('mesh')
                        if mesh is not None:
                            mesh_path = mesh.get('filename')
                            origin = visual.find('origin')
                            xyz = [0.0, 0.0, 0.0]
                            rpy = [0.0, 0.0, 0.0]
                            if origin is not None:
                                xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()]
                                rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()]
                            self.visuals[link_name].append((mesh_path, xyz, rpy))
        except Exception as e:
            self.get_logger().error(f"Failed to load URDF: {e}")

    def motor_status_callback(self, msg):
        self.last_data_time = time.time()
        for status in msg.status:
            motor_id = status.name # name field stores the ID (uint16)
            if motor_id in MOTOR_ID_TO_JOINT:
                joint_name = MOTOR_ID_TO_JOINT[motor_id]
                self.joint_positions[joint_name] = status.pos

    def handle_hand_callback(self, msg: JointState, hand_map):
        min_len = min(len(msg.position), 6)
        for i in range(min_len):
            motor_id = i + 1
            if motor_id in hand_map:
                joint_name, limit = hand_map[motor_id]
                percentage = msg.position[i]
                rad = (1.0 - percentage) * limit 
                self.joint_positions[joint_name] = rad

    def left_hand_callback(self, msg):
        self.last_data_time = time.time()
        self.handle_hand_callback(msg, LEFT_HAND_JOINTS)

    def right_hand_callback(self, msg):
        self.last_data_time = time.time()
        self.handle_hand_callback(msg, RIGHT_HAND_JOINTS)

    def publish_joint_states(self):
        # Publish connection status
        current_time = time.time()
        is_online = (current_time - self.last_data_time) < 1.0
        
        status_msg = Bool()
        status_msg.data = is_online
        self.status_pub_.publish(status_msg)

        # Publish Status Marker - Text Display disabled by user
        marker = Marker()
        marker.header.frame_id = "pelvis"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "connection_status"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.DELETE
        self.marker_pub_.publish(marker)

        if is_online:
            # Clear offline red robot
            ma = MarkerArray()
            # We can't easily delete all by ID range without tracking, but we can delete all namespaces?
            # Or just send DELETEALL action if supported (ROS 2 Humble Rviz might support it but it's safer to use DELETE with known IDs)
            # Simplest: Just publish nothing or empty marker with DELETEALL action if rviz supports it. 
            # Marker.DELETEALL is 3
            del_marker = Marker()
            del_marker.action = 3 # DELETEALL
            ma.markers.append(del_marker)
            self.offline_visual_pub_.publish(ma)
            
        else:
            # Publish Red Robot Overlay
            ma = MarkerArray()
            idx = 0
            for link_name, visuals in self.visuals.items():
                for (mesh_path, xyz, rpy) in visuals:
                    m = Marker()
                    m.header.frame_id = link_name
                    m.header.stamp = self.get_clock().now().to_msg()
                    m.ns = "offline_robot"
                    m.id = idx
                    idx += 1
                    m.type = Marker.MESH_RESOURCE
                    m.action = Marker.ADD
                    m.mesh_resource = mesh_path
                    m.mesh_use_embedded_materials = False
                    
                    m.color.r = 1.0
                    m.color.g = 0.0
                    m.color.b = 0.0
                    m.color.a = 0.7 # Semi-transparent Red
                    
                    m.scale.x = 1.005 # Slightly larger to cover original?
                    m.scale.y = 1.005
                    m.scale.z = 1.005
                    
                    # Pose offset from visual origin
                    qx, qy, qz, qw = euler_to_quaternion(rpy[0], rpy[1], rpy[2])
                    m.pose.position.x = xyz[0]
                    m.pose.position.y = xyz[1]
                    m.pose.position.z = xyz[2]
                    m.pose.orientation.x = qx
                    m.pose.orientation.y = qy
                    m.pose.orientation.z = qz
                    m.pose.orientation.w = qw
                    
                    ma.markers.append(m)
            self.offline_visual_pub_.publish(ma)
        
        # Always publish joint states for visualization

        # Always publish joint states for visualization
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []
        
        for name, pos in self.joint_positions.items():
            msg.name.append(name)
            msg.position.append(pos)
          
        self.publisher_.publish(msg)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = JointStatePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # These guard against pre-init interruptions
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
