import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import csv



LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
LIDAR_WALL_DISTANCE = 0.45
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX=270  
RIGHT_FRONT_INDEX=330 # 210
LEFT_FRONT_INDEX=30 # 150
LEFT_SIDE_INDEX=90
LEFT_MIDDLE_INDEX=60 # 120
LEFT_BACK_INDEX=135 # 45
ROTATION_WAIT=30
ROTATION_LENGTH=24
MOVEMENT_FACTOR=0.7
ROTATION_FACTOR=0.5

class WallFollow(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('wall_follow_node')
        # self.dist = 0
        # self.max_dist = 0
        self.scan_cleaned = []
        self.stall = 0
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # self.laser_forward = 0
        # self.odom_data = 0
        timer_period = 0.5
        # self.pose_saved=''
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.position_log = []
        self.position_it = 0
        self.stall_idx = -1
        self.rotation_wait = ROTATION_WAIT   
        self.rotatetimer = 0
        

    # Clean lidar readings
    def listener_callback1(self, msg1):
        scan = msg1.ranges
        self.scan_cleaned = []
       
        # Assume 360 range measurements
        for reading in scan:
            if reading == float(0.0) or reading == float('inf'):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
                self.scan_cleaned.append(reading)


    # Record positions
    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        (posx, posy, posz) = (position.x, position.y, position.z)
        #self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz))

        if self.position_it % 10 == 0:
            self.position_log.append([posx, posy, posz])
            self.position_it = 0
        self.position_it += 1
        
        prev_pos = self.position_log[0]
        if len(self.position_log) > 7:
            prev_pos = self.position_log[-7]
        diffX = math.fabs(prev_pos[0] - posx)
        diffY = math.fabs(prev_pos[1] - posy)

        # Checks for a stall
        if len(self.position_log) > 10 and diffX < 0.1 and diffY < 0.1:
           if not self.stall:                                   # Begin stall recovery 1
               self.get_logger().info('Entering stall')
           self.stall = 1
           self.stall_idx = len(self.position_log)
        elif len(self.position_log) > self.stall_idx + 10:      # Begin stall recovery 2
            self.stall = 2
        elif not self.stall or len(self.position_log) > self.stall_idx + 30:
           if self.stall:
               self.get_logger().info('Ending stall')
           self.stall = 0
           
        return None
        
    def timer_callback(self):
        if (len(self.scan_cleaned)==0):
            self.turtlebot_moving = False
            return

        front_lidar_left = min(self.scan_cleaned[0:30])
        front_lidar_right = min(self.scan_cleaned[330:])
        front_lidar_min = min(front_lidar_left, front_lidar_right)
        
        left_lidar_back = min(self.scan_cleaned[LEFT_BACK_INDEX-20:LEFT_BACK_INDEX+20])
        left_lidar_side = min(self.scan_cleaned[LEFT_SIDE_INDEX-20:LEFT_SIDE_INDEX+20])
        
        left_lidar_middle = min(self.scan_cleaned[LEFT_FRONT_INDEX:LEFT_MIDDLE_INDEX]) # LEFT_MIDDLE_INDEX:LEFT_FRONT_INDEX

        
        #if self.rotation_wait == 0:
            #self.rotatetimer = ROTATION_LENGTH
        #if self.rotation_wait >= 0:
            #self.rotation_wait -= 1
        #self.get_logger().warning(f'Time til 360: {self.rotation_wait}')
        
        

            
        if front_lidar_min < SAFE_STOP_DISTANCE: # Stopping
            if self.turtlebot_moving == True:
                self.cmd.linear.x = 0.0 * MOVEMENT_FACTOR
                self.cmd.angular.z = 0.0 * ROTATION_FACTOR
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = False
                self.get_logger().info('Stopping')
                return
        #elif self.rotatetimer >= 1:
            #self.cmd.angular.z = -1.0 * ROTATION_FACTOR
            #self.cmd.linear.x = 0.0
            #self.rotatetimer -= 1
            #self.publisher_.publish(self.cmd)
            #self.get_logger().warning(f'Time left in 360: {self.rotatetimer}')
            #if self.rotatetimer == 0:
                #self.rotation_wait = ROTATION_WAIT
            #return
        elif self.stall and front_lidar_min < LIDAR_WALL_DISTANCE and left_lidar_middle < LIDAR_WALL_DISTANCE: # Stall Recovery
            if self.stall == 1: # First, reverse to the right
                self.cmd.linear.x = -0.1 * MOVEMENT_FACTOR
                self.cmd.angular.z = -1.0 * ROTATION_FACTOR
            else: # Next, move forward to the right
                self.cmd.linear.x = 0.15 * MOVEMENT_FACTOR
                self.cmd.angular.z = -1.1 * ROTATION_FACTOR
            self.publisher_.publish(self.cmd)
            self.get_logger().info('De-stalling')
        elif front_lidar_min < LIDAR_AVOID_DISTANCE: 
            self.cmd.linear.x = 0.04 * MOVEMENT_FACTOR
            if left_lidar_middle > 1.3 * left_lidar_side and left_lidar_side > LIDAR_WALL_DISTANCE and front_lidar_min > LIDAR_WALL_DISTANCE:
                self.cmd.angular.z = 0.25 * ROTATION_FACTOR
                self.get_logger().warning('rotating left')
            else:
                self.cmd.angular.z = -0.3 * ROTATION_FACTOR
                self.get_logger().warning('rotating right')
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
        elif left_lidar_back < front_lidar_min and (left_lidar_back < left_lidar_side or left_lidar_side < left_lidar_middle) and min(left_lidar_middle, left_lidar_side) > LIDAR_WALL_DISTANCE:
            self.cmd.linear.x = 0.04 * MOVEMENT_FACTOR
            self.cmd.angular.z = 0.4 * ROTATION_FACTOR
            self.get_logger().warning('rotating slightly more left')
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
        elif left_lidar_side <= front_lidar_min and min(left_lidar_side, left_lidar_middle) > LIDAR_WALL_DISTANCE: # Left adjust
            self.cmd.linear.x = 0.15 * MOVEMENT_FACTOR
            self.cmd.angular.z = 0.25 * ROTATION_FACTOR
            self.get_logger().warning('rotating slightly left')
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
        else: # Basic straightforward movement
            self.cmd.linear.x = 0.3 * MOVEMENT_FACTOR
            self.cmd.angular.z = 0.0 * ROTATION_FACTOR
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Not Turning')
            self.turtlebot_moving = True
            

        self.get_logger().info('Distance of the obstacle : %f' % front_lidar_min)
        # self.get_logger().info('Distance from the starting point: %f' % self.dist)

        # if self.stall == True:
        #    self.get_logger().info('Stall reported')
 


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    wall_follow_node = WallFollow()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(wall_follow_node)
    # writes the location data to a csv file
    with open('position_log.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(wall_follow_node.position_log)
    
    # Explicity destroy the node
    wall_follow_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()



if __name__ == '__main__':
    main()
