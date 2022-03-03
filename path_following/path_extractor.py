import time, os
from ament_index_python.packages import get_package_share_directory
from math import pi, cos, sin, pi, sqrt, pow

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point 
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

# HOST = '192.168.0.19'
# PORT = 9008
# 소켓 통신 정보

class PathExtracter(Node):

    def __init__(self):
        super().__init__('path_extractor')

        # self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.sock.connect((HOST, PORT))
        
        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.path_rviz_pub = self.create_publisher(Path, "/path_rviz", QOS_RKL10V)
        
        self.path_subscriber = self.create_subscription(PoseStamped, '/RigidBody001/pose',
            self.subscribe_topic_message,QOS_RKL10V)

        self.path_msg = Path()
        self.point = PoseStamped()
        self.prev_point = PoseStamped()

        self.file_open()

        self.point.pose.position.x = 0.0
        self.point.pose.position.y = 0.0
        self.point.pose.position.z = 0.0
        self.point.pose.orientation.x = 0.0
        self.point.pose.orientation.y = 0.0
        self.point.pose.orientation.z = 0.0
        self.point.pose.orientation.w = 1.0

        self.prev_point.pose.position.x = 0.0
        self.prev_point.pose.position.y = 0.0
        self.prev_point.pose.position.z = 0.0
        self.prev_point.pose.orientation.x = 0.0
        self.prev_point.pose.orientation.y = 0.0
        self.prev_point.pose.orientation.z = 0.0
        self.prev_point.pose.orientation.w = 1.0

        self.prev_x = 0.0
        self.prev_y = 0.0

    def subscribe_topic_message(self, msg): 
        self.point.header = msg.header
        self.point.pose.position.x = msg.pose.position.x
        self.point.pose.position.y = msg.pose.position.y
        self.point.pose.position.z = msg.pose.position.z
        self.point.pose.orientation.x = msg.pose.orientation.x
        self.point.pose.orientation.y = msg.pose.orientation.y
        self.point.pose.orientation.z = msg.pose.orientation.z
        self.point.pose.orientation.w = msg.pose.orientation.w

        # self.time = msg.header.stamp.sec

    def prev(self):
        self.prev_x = self.point.pose.position.x 
        self.prev_y = self.point.pose.position.y

    def file_open(self):
        pkg_share_path = get_package_share_directory('path_following')
        path = pkg_share_path[:-43] + "src/path_following/path"
        file = path + "/path.txt"

        os.makedirs(path, exist_ok=True) # 디렉토리 없으면 만들기
        
        self.f = open(file, 'w')

    def file_save(self):
        self.limit = 0.1 # 측정위한 최소 거리
        self.p_x = self.point.pose.position.x
        self.p_y = self.point.pose.position.y
        self.p_z = self.point.pose.position.z
        self.o_x = self.point.pose.orientation.x
        self.o_y = self.point.pose.orientation.y
        self.o_z = self.point.pose.orientation.z
        self.o_w = self.point.pose.orientation.w

        # print('------------------------------')
        # print(self.time)

        # self.socket_pub()
        
        self.distance = sqrt(pow(self.p_x - self.prev_x, 2) + pow(self.p_y - self.prev_y, 2))
        
        if(self.limit < self.distance):
            print("recorded")
            print(self.point.pose.position.x, self.point.pose.position.y, self.point.pose.position.z, self.o_w)

            data = '{0}\t{1}\t{2}\n'.format(self.point.pose.position.x, self.point.pose.position.y, self.point.pose.position.z)
            self.f.write(data)

            save_point = PoseStamped()
            save_point.pose.position.x = self.p_x
            save_point.pose.position.y = self.p_y
            save_point.pose.position.z = self.p_z
            save_point.pose.orientation.x = self.o_x
            save_point.pose.orientation.y = self.o_y
            save_point.pose.orientation.z = self.o_z
            save_point.pose.orientation.w = self.o_w

            self.path_msg.header = self.point.header
            self.path_msg.poses.append(save_point)
            self.prev()

            self.path_rviz_pub.publish(self.path_msg)

    # def socket_pub(self):

    #     self.socket_data = f"{self.p_x}/{self.p_y}/{self.p_z}/{self.o_x}/{self.o_y}/{self.o_z}/{self.o_w}"
    #     self.sock.send(self.socket_data.encode())
  
def main(args=None):
    rclpy.init(args=args)
    node = PathExtracter()
    try:
        spin_count = 0
        while rclpy.ok():
            # spin_count += 1
            # print('spin_count: ' + str(spin_count))
            rclpy.spin_once(node, timeout_sec=0.1)
            node.file_save()

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')

    finally:
        node.f.close()
        # node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
