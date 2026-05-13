import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from xtd2_msgs.srv import XTD2Cmd
import time

class AutoMission(Node):
    def __init__(self):
        super().__init__('auto_mission')
        # 控制消息发布
        self.cmd_vel_pub = self.create_publisher(Twist, '/xtdrone2/x500_0/cmd_vel_ned', 10)
        self.cmd_pose_pub = self.create_publisher(Pose, '/xtdrone2/x500_0/cmd_pose_local_ned', 10)
        # 服务客户端
        self.offboard_cli = self.create_client(XTD2Cmd, '/xtdrone2/x500_0/cmd')
        while not self.offboard_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('waiting for service...')
    
    def call_service(self, cmd):
        req = XTD2Cmd.Request()
        req.command = cmd
        future = self.offboard_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def pub_pose(self, x, y, z, seconds):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        for _ in range(int(seconds * 10)):
            self.cmd_pose_pub.publish(pose)
            time.sleep(0.1)

    def pub_vel(self, x, y, z, seconds):
        tw = Twist()
        tw.linear.x = x
        tw.linear.y = y
        tw.linear.z = z
        for _ in range(int(seconds * 10)):
            self.cmd_vel_pub.publish(tw)
            time.sleep(0.1)

    def run(self):
        # [1] 发送setpoint让PX4能进OFFBOARD（心跳占坑，速度为0，可选）
        self.pub_vel(0,0,0, 2)

        # [2] 切到OFFBOARD模式
        assert self.call_service('OFFBOARD')
        assert self.call_service('ARM')

        # [3] 起飞到2米
        self.pub_pose(0,0,-2.0, 6)

        # [4] 向前平移
        self.pub_vel(1.0,0,0, 5)

        # [5] 再飞到另一个点
        self.pub_pose(3.0, 2.0, -2.0, 8)

        # [6] 最后降落/结束（如需可写）
        # self.pub_pose(3.0, 2.0, 0.0, 8)

if __name__ == '__main__':
    rclpy.init()
    node = AutoMission()
    node.run()
    node.destroy_node()
    rclpy.shutdown()
