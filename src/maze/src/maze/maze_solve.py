#!/usr/bin/env python3
import rospy
from threading import Thread, Lock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from pid_controller import PID
import time as time
import actionlib
from maze.srv import det_srv,solver
from maze.msg import m_goal,mazeSolverAction, mazeSolverGoal


def wall_det_service(flag,actMinLaserValue,turnSpeed):
    '''
    Service Client for wall detection
    '''
    rospy.wait_for_service('mazesolver/wall_detection')
    try:
        handle = rospy.ServiceProxy('mazesolver/wall_detection', det_srv)
        resp = handle(flag,actMinLaserValue,turnSpeed)
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed %s", e)


def action_client():
    '''
    Action Client. This will keep running until the bot exits the maze i.e., reaches the goal.
    '''
    client = actionlib.SimpleActionClient('action_server_node', mazeSolverAction)
    client.wait_for_server()
    action_goal = mazeSolverGoal()
    action_goal.goal = 1
    client.send_goal(action_goal)
    client.wait_for_result()
    return client.get_result()


class MazeSolver:

    def __init__(self):
        """
        Constructor for MazeSolver class
        """
        self.rate = rospy.Rate(10)
        self.initialize_sub_pub()
        self.initialize_params()
        self.driveState = "WallDetection"
        self.pid = PID(self.kp, self.ki, self.kd, self.outMin, self.outMax, self.iMin, self.iMax)  # Initializing PID
        rospy.Timer(rospy.Duration(0.1), self.startSolver)

    def initialize_sub_pub(self):
        """
        Initialize the subscribers and publishers
        """
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 5)
        self.goal_pub = rospy.Publisher('/mazesolver/goal', m_goal, queue_size = 1)

    def initialize_params(self):
        """
        Initialize the parameters
        """
        self.vel = Twist()
        self.maze_goal = m_goal()
        self._flag = False
        self.laser = None
        self.odom = None
        self.intervalEpsilon = 1.5              # Upper and Lower bounds of the laser
        self.mutex2 = Lock()
        self.knownPoints = []
        self.epsilonAroundPoints = 0.1          # Used to detect if the bot is circling around the same path
        self.timeoutForDetection = 12
        self.laserIndex = 90                   # Left laser index
        self.turnSpeed = -0.3
        self.distanceToWall = 0.5               # Distance between wall and bot
        self.counter = 0
        self.angle = 1.57                       # 90 Degrees
        self.minLasersSide = [-20, 20]          # Used for obstacle detection
        self.mutex = Lock()
        self.kp = 6
        self.ki = 4
        self.kd = 1.5
        self.outMin = -0.4
        self.outMax = 0.4
        self.iMin = -0.4
        self.iMax = 0.4

    def odom_callback(self, odom_msg):
        """
        Callback function for the odometry
        """
        self.odom = odom_msg

    def laserscan_callback(self, laser_msg):
        """
        Callback function for the laser scan
        """
        self.laser = laser_msg

    def startSolver(self, event):
        """
        Function to start the maze solver
        Args:
            event: Phase of the solver
        """
        if self._flag:                           # If service is called, the bot starts solving for maze

            if(self.laser and self.odom):

                # Storing the origin of the bot
                if(len(self.knownPoints) == 0):
                    self.knownPoints.append([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, time.time()])

                # Minimum laser value for obstacle detection infront of the robot
                actMinLaserValue = min(min(self.laser.ranges[self.minLasersSide[0]:]), min(self.laser.ranges[:self.minLasersSide[1]]))

                # If there are no objects in the vicinity
                if actMinLaserValue>20:
                    self.counter +=1

                # Bot has reached the goal successfully if the laser value is large for a certain duration of 500 iterations
                if self.counter>500:
                    self.maze_goal.res = 1
                    self.goal_pub.publish(self.maze_goal)   #Publishes that the bot has reached the goal

                else:

                    if(self.driveState == "WallDetection"):
                        self.vel.linear.x = 0.0
                        self.vel.angular.z = 0.0
                        flag = wall_det_service(True,actMinLaserValue,self.turnSpeed)      # Calls the service to detect for walls
                        if flag:
                            self.driveState = "driveToWall"                                # Wall has been detected; Drive to the wall
                        self.pid.resetValues()

                    elif(self.driveState == "driveToWall"):
                        if(self.mutex.locked() == False):
                            if(actMinLaserValue <= self.distanceToWall):                                                                       # Either the bot has reached the wall or an obstacle is found in front of the bot
                                self.knownPoints.append([self.odom.pose.pose.position.x,self.odom.pose.pose.position.y, time.time()])          # Storing the actual position of the bot for loop detection
                                self.vel.linear.x = 0.0
                                self.vel.angular.z = 0.0
                                self.rotate_angle(self.angle, self.turnSpeed)                                                                  # Turn and follow
                                self.driveState = "WallFollow"
                            else:
                                self.vel.linear.x = 0.5                                                                                        # Else drive to the wall
                                self.vel.angular.z = 0.0

                    elif(self.driveState == "WallFollow"):
                        if(self.mutex.locked() == False):

                            # Checking the traversed points to detect loops
                            for i in range(0, len(self.knownPoints)):
                                if(self.odom.pose.pose.position.x - self.epsilonAroundPoints <= self.knownPoints[i][0] <= self.odom.pose.pose.position.x + self.epsilonAroundPoints and
                                self.odom.pose.pose.position.y - self.epsilonAroundPoints <= self.knownPoints[i][1] <= self.odom.pose.pose.position.y + self.epsilonAroundPoints and
                                self.knownPoints[i][2] + self.timeoutForDetection <  time.time()):
                                    rospy.loginfo("Loop detected")
                                    self.driveState = "WallDetection"

                            # Else follow the wall
                            self.wallFollower(actMinLaserValue)

                    self.maze_goal.res = 0
                    self.goal_pub.publish(self.maze_goal)  #Publishes that the bot has not reached the goal


                self.vel_pub.publish(self.vel)

            self.rate.sleep()


    def wallFollower(self, actMinLaserValue):
        """
        Function to follow the wall once detected

        Args:
            actMinLaserValue: Minimum laser value
        """
        try:
            pidValue = self.pid.pidExecute(self.distanceToWall, self.laser.ranges[self.laserIndex]) * -1
            # The pidValue is multiplied by -1 to make the bot follow the wall on the left side


            # Obstacle in front of the bot
            if(actMinLaserValue < self.distanceToWall):
                self.rotate_angle(self.angle, self.turnSpeed)
            if(pidValue == 0):
                self.vel.linear.x = 0.5
                self.vel.angular.z = 0.0
            elif(pidValue != 0):
                self.vel.linear.x = 0.3
                self.vel.angular.z = pidValue

        except Exception as e:
            rospy.logerr_throttle(10, "Wall Follower Error")

    def rotate_angle(self, angle, speed):
        """
        Function to rotate the bot by a certain angle at a certain speed

        Args:
            angle: desired angle
            speed: desired speed
        """
        self.mutex.acquire()
        try:
            angular_speed = speed
            relative_angle = angle
            self.vel.linear.x = 0.0
            self.vel.angular.z = angular_speed

            t0 = rospy.Time.now().to_sec()
            current_angle = 0
            #Rotate the bot
            while(current_angle < relative_angle):
                self.vel_pub.publish(self.vel)
                t1 = rospy.Time.now().to_sec()
                current_angle = abs(angular_speed)*(t1-t0)
            #Bot stops after turning
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0
            self.vel_pub.publish(self.vel)
            self.pid.resetValues()
        finally:
            self.mutex.release()


    def serviceCB(self,req):
        '''
        Service to start solving the maze.
        It also invokes a goal based action server.
        '''
        if req.solver_flag:
            self._flag = True
            rospy.loginfo("Mission Started")
            result = action_client()
            if result:
                self._flag = False
                rospy.loginfo("Mission Accomplished!!!")
            return True
        else:
            self._flag=False
            rospy.loginfo("Mission Stopped")
            return True


def main():
    rospy.init_node('Maze Solver', anonymous=True)
    obj = MazeSolver()
    s = rospy.Service('/mazesolver', solver, obj.serviceCB)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down the node")


if __name__ == '__main__':
    main()
