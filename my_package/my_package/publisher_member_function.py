import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint

class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        # publish_topic = "/joint_trajectory_controller/joint_trajectory"
        publish_topic = "/joint_trajectory_controller/joint_trajectory"
        
        self.trajectory_publihser = self.create_publisher(JointTrajectory,publish_topic, 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joints = ['joint1','joint2','joint3','joint4','joint5','joint6']
        self.goal_positions = [0.1,0.1,0.1,0.1,0.1,0.1]
        
        self.goal_positions = [1.530869483947754, -0.5006757616996765, -0.01343208272010088, -0.39744195342063904, -1.68803071975708, 0.001489347661845386]
        self.goal_velocities = [0.1,0.1,0.1,0.1,0.1,0.1]
        self.goal_accelerations = [1.5,1.5,1.5,1.5,1.3,1.1]


    def timer_callback(self):
        bazu_trajectory_msg = JointTrajectory()
        bazu_trajectory_msg.joint_names = self.joints
        ## creating a point
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        # point.velocities = self.goal_velocities
        # point.accelerations = self.goal_accelerations
        point.time_from_start = Duration(sec=5)
        ## adding newly created point into trajectory message
        bazu_trajectory_msg.points.append(point)
        self.trajectory_publihser.publish(bazu_trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()

    rclpy.spin(joint_trajectory_object)
    joint_trajectory_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# class MinimalPublisher(Node):

#     def __init__(self):
#         super().__init__('minimal_publisher')
#         self.publisher_ = self.create_publisher(String, 'topic', 10)
#         timer_period = 0.5  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.i = 0

#     def timer_callback(self):
#         msg = String()
#         msg.data = 'Hello World: %d' % self.i
#         self.publisher_.publish(msg)
#         self.get_logger().info('Publishing: "%s"' % msg.data)
#         self.i += 1


# def main(args=None):
#     rclpy.init(args=args)

#     minimal_publisher = MinimalPublisher()

#     rclpy.spin(minimal_publisher)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     minimal_publisher.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()