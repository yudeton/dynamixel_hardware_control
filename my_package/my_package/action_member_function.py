import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.action import ActionClient
# from std_msgs.msg import String
# from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration, Time
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
from openpyxl import load_workbook
import os
curr_path = os.path.dirname(os.path.abspath(__file__))  # 当前文件所在绝对路径

class JointTrajectoryActionClient():

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.point_test_excel = "./src/my_package/traj_excel_30/tested_robot_traj_2_.xlsx"
        self.traj_file = ["./src/my_package/traj_excel_30/tested_robot_traj_1_.xlsx",
                    "./src/my_package/traj_excel_30/tested_robot_traj_2_.xlsx",
                    "./src/my_package/traj_excel_30/tested_robot_traj_3_.xlsx",
                    "./src/my_package/traj_excel_30/tested_robot_traj_4_.xlsx",
                    "./src/my_package/traj_excel_30/tested_robot_traj_5_.xlsx",
                    "./src/my_package/traj_excel_30/tested_robot_traj_6_.xlsx",
                    "./src/my_package/traj_excel_30/tested_robot_traj_7_.xlsx"]
        # self.traj_file = ["./src/my_package/traj_excel_10/tested_robot_traj_1_.xlsx",
        #             "./src/my_package/traj_excel_10/tested_robot_traj_2_.xlsx",
        #             "./src/my_package/traj_excel_10/tested_robot_traj_3_.xlsx",
        #             "./src/my_package/traj_excel_10/tested_robot_traj_4_.xlsx",
        #             "./src/my_package/traj_excel_10/tested_robot_traj_5_.xlsx",
        #             "./src/my_package/traj_excel_10/tested_robot_traj_6_.xlsx",
        #             "./src/my_package/traj_excel_10/tested_robot_traj_7_.xlsx"]
        # self.traj_file = ["./src/my_package/traj_excel_30_v2/tested_robot_traj_1_.xlsx",
        #             "./src/my_package/traj_excel_30_v2/tested_robot_traj_2_.xlsx",
        #             "./src/my_package/traj_excel_30_v2/tested_robot_traj_3_.xlsx",
        #             "./src/my_package/traj_excel_30_v2/tested_robot_traj_4_.xlsx",
        #             "./src/my_package/traj_excel_30_v2/tested_robot_traj_5_.xlsx",
        #             "./src/my_package/traj_excel_30_v2/tested_robot_traj_6_.xlsx",
        #             "./src/my_package/traj_excel_30_v2/tested_robot_traj_7_.xlsx"]
        self.current_path = curr_path
        self._action_client = ActionClient(self.node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.joints = ['joint1','joint2','joint3','joint4','joint5','joint6']
        self.goal_positions = [-0.202886489675801, 1.56825683117444, 1.37931678536383, 1.38185643789439, -1.57079638404643, -1.36790988828724]
        self.goal_positions_traj = [0,0,0,0,0,0]
        self.total_duration_time = 0  # 初始化总的duration_time
        self.tmp_time = 0
        self.result_successful = 0
        self.get_result_flag = False
        self.traj_time = 0
        
    def send_goal(self, traj_file):
        traj_position, duration_time = self.load_traj(traj_file)
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joints
        self.JointTrajectoryPoint_append(trajectory_msg, traj_position, duration_time)
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.node.get_logger().info('send goal :)')
        return self._send_goal_future
    def send_init_goal(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joints
        ## creating a point
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=10)
        trajectory_msg.points.append(point)
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.node.get_logger().info('send init goal :)')
        
        return self._send_goal_future
    def load_traj(self, load_traj_file):
        df = load_workbook(load_traj_file)
        sheets = df.worksheets
        sheet1 = sheets[0]
        rows = sheet1.rows
        goal_positions_traj = []
        goal_time = []
        for row in rows:
            row_val = [col.value for col in row]
            goal_positions_traj.append([row_val[1], row_val[2], row_val[3],row_val[4], row_val[5], row_val[6]])
            goal_time.append(row_val[0])
        return goal_positions_traj, goal_time
    def get_ros_time(self):
        current_time = self.node.get_clock().now()  # 获取ROS时间
        return current_time

    def JointTrajectoryPoint_append(self,traj_msg,goal, time):
        # print("goal_time:", time)
        for i in range(len(goal)):
            point = JointTrajectoryPoint()
            point.positions = goal[i]
            dur_time = time[i]
            seconds = int(dur_time)  # 获取整数部分
            nanoseconds = int((dur_time - seconds) * 1e9)  # 获取小数部分并转换为纳秒
            point.time_from_start = Duration(sec = seconds, nanosec = nanoseconds)
            traj_msg.points.append(point)

    def goal_response_callback(self,future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected :(')
            return
        self.node.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    def get_result_callback(self, future):
        result = future.result().result
        self.node.get_logger().info('Result: {0}'.format(result.SUCCESSFUL))
        self.result_successful = result.error_code
        self.get_result_flag = True
        self.traj_time = self.traj_time +1
        self.node.get_logger().info('self.traj_time:{0}'.format(self.traj_time))
        self.node.get_logger().info('result_successful:{0}'.format(self.result_successful))

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('your_node')
    action_client = JointTrajectoryActionClient(node)
    
    
    future = action_client.send_init_goal()
    rclpy.spin_until_future_complete(node,future)
    # 阻塞等待结果返回
    if future.result():
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        # result = result_future.result().result
        # print('Received result:', result)
    else:
        print('Action request failed')
    for i in range(len(action_client.traj_file)):
        future = action_client.send_goal(action_client.traj_file[i])
        rclpy.spin_until_future_complete(node,future)
        # 阻塞等待结果返回
        if future.result():
            goal_handle = future.result()
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future)
            # result = result_future.result().result
            # print('Received result:', result)
        else:
            print('Action request failed')
            node.destroy_node()
            rclpy.shutdown()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()