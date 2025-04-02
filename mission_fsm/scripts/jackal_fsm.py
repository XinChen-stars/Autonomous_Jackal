#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Int16
from geometry_msgs.msg import Pose, PoseStamped, Twist

# state machine
fsm = ['INITIALIZING', 'WAITING_FOR_GOAL', 'MOVING_TO_GOAL','WAITING_FOR_BARREL','MOVING_TO_BARREL','PASSING_THE_BRIDGE', 'FINISHED']
waypoints = [
    [21.5, 0.0, 0.0],
    [21.5, -21.5, 0.0],
    [19.0, -21.5, 0.0],
    [19.0, -3.0, 0.0],
    [10.0, -3.0, 0.0],
    [10.0, -21.5, 0.0],
]


class Jackal_FSM():
    def __init__(self):
        super().__init__()
        self.position = None
        self.orientation = None
        self.goal = None
        self.goal_id = 0
        self.barrel_waypoint_1 = None
        self.barrel_waypoint_2 = None
        self.state = "INITIALIZING"
        self.recieived_barrel = False

    def odom_callback(self, data):
        # Process odometry data
        self.position = data.pose.pose.position
        self.orientation = data.pose.pose.orientation
        # rospy.loginfo(f"Position: {self.position}, Orientation: {self.orientation}")

    def barrel_waypoint_callback(self, data):
        if data.pose.position.x >= 10.0:
            # Check if the waypoint is valid
            rospy.logwarn("Invalid waypoint received")
            return
        if self.recieived_barrel:
            # Check if the waypoint is already received
            rospy.logwarn("Barrel waypoint already received")
            return
        self.recieived_barrel = True
        self.barrel_waypoint_1 = data.pose.position
        self.barrel_waypoint_1.x += 2.5
        self.barrel_waypoint_2 = data.pose.position
        self.barrel_waypoint_2.x -= 3.0
        rospy.loginfo(f"Barrel waypoint received: {self.barrel_waypoint_1}")

    def pub_goal(self, goal):
        # Publish the goal to the local planner
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.pose.position = goal.position
        goal_msg.pose.orientation = goal.orientation
        self.goal_point_pub.publish(goal_msg)
        rospy.loginfo(f"Goal published: {goal}")

    def pub_respawn(self):
        # Publish respawn command
        respawn_msg = Int16()
        respawn_msg.data = 1
        self.respawn_pub.publish(respawn_msg)
        rospy.loginfo("Respawn command published")
    
    def pub_open_bridge(self):
        # Publish open bridge command
        open_bridge_msg = Bool()
        open_bridge_msg.data = True
        self.open_bridge_pub.publish(open_bridge_msg)
        rospy.loginfo("Open bridge command published")

    def is_reached_goal(self):
        # Check if the robot has reached the goal
        if self.position is not None and self.goal is not None:
            distance = ((self.position.x - self.goal.position.x) ** 2 + (self.position.y - self.goal.position.y) ** 2) ** 0.5
            return distance < 0.5
    
    def select_goal(self,id):
        # Select a goal based on the given ID
        if id < len(waypoints):
            self.goal = Pose()
            self.goal.position.x = waypoints[id][0]
            self.goal.position.y = waypoints[id][1]
            self.goal.position.z = 0.0
            self.goal.orientation.x = 0.0
            self.goal.orientation.y = 0.0
            self.goal.orientation.z = 0.0
            self.goal.orientation.w = 1.0
            rospy.loginfo(f"Goal selected: {self.goal}")
        else:
            rospy.logwarn(f"Invalid goal ID: {id}")
            self.goal = None


    def transition_state(self, new_state):
        # Transition to a new state
        if new_state in fsm:
            self.state = new_state
            rospy.loginfo(f"Transitioning to state: {self.state}")
        else:
            rospy.logwarn(f"Invalid state transition attempted: {new_state}")

    def init(self, args=None):
        # Initialize the state
        self.position = None
        self.orientation = None
        self.goal = None
        self.state = "INITIALIZING"
        
        # Initialize publishers and subscribers
        self.goal_point_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.respawn_pub = rospy.Publisher('/rviz_panel/respawn_objects', Int16, queue_size=10) 
        self.open_bridge_pub = rospy.Publisher('/cmd_open_bridge', Bool, queue_size=10)
        
        # Subscriber for receiving odometry data
        self.odom_sub = rospy.Subscriber('/Odometry', Odometry, self.odom_callback)
        self.barrel_waypoint_sub = rospy.Subscriber('/barrel_waypoint', PoseStamped, self.barrel_waypoint_callback)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('jackal_state')

    # Create an instance of the Jackal class
    FSM = Jackal_FSM()
    
    # Initialize the state
    FSM.init()
    rospy.sleep(1)  # wait for the publishers to be ready

    # Start the FSM loop
    rate = rospy.Rate(20)  # 20 Hz
    while not rospy.is_shutdown():
        # Main loop of the FSM
        if FSM.state == "INITIALIZING":
            FSM.pub_respawn()
            # Set the initial goal
            FSM.select_goal(FSM.goal_id)
            FSM.goal_id += 1
            # Transition to WAITING_FOR_GOAL state
            FSM.transition_state("WAITING_FOR_GOAL")
        
        elif FSM.state == "WAITING_FOR_GOAL":
            if FSM.goal is not None:
                FSM.pub_goal(FSM.goal)
                FSM.transition_state("MOVING_TO_GOAL")
        
        elif FSM.state == "MOVING_TO_GOAL":
            # Move towards the goal
            # Check if the robot has reached the goal
            if FSM.is_reached_goal():
                # change the goal
                FSM.select_goal(FSM.goal_id)
                FSM.goal_id += 1
                if FSM.goal_id > len(waypoints):
                    FSM.transition_state("WAITING_FOR_BARREL")
                else:
                    FSM.transition_state("WAITING_FOR_GOAL")
            else:
                print("Moving towards goal...")
                # Continue moving towards the goal
                # FSM.pub_goal(FSM.goal)

        elif FSM.state == "WAITING_FOR_BARREL":
            if FSM.recieived_barrel:
                # Check if the barrel waypoint is received
                FSM.goal.position.x = FSM.barrel_waypoint_1.x
                FSM.goal.position.y = FSM.barrel_waypoint_1.y
                FSM.goal.position.z = 0.0
                FSM.pub_goal(FSM.goal)
                FSM.transition_state("MOVING_TO_BARREL")
            else:
                rospy.loginfo("Waiting for barrel waypoint...")
        
        elif FSM.state == "MOVING_TO_BARREL":
            # Move towards the barrel waypoint
            if FSM.is_reached_goal():
                # Check if the robot has reached the barrel waypoint
                FSM.pub_open_bridge()
                FSM.goal.position.x = FSM.barrel_waypoint_2.x
                FSM.goal.position.y = FSM.barrel_waypoint_2.y
                FSM.goal.position.z = 0.0
                FSM.pub_goal(FSM.goal)
                FSM.transition_state("PASSING_THE_BRIDGE")
            else:
                print("Moving towards barrel waypoint...")
                # Continue moving towards the barrel waypoint

        elif FSM.state == "PASSING_THE_BRIDGE":
            if FSM.is_reached_goal():
                # Check if the robot has reached the barrel waypoint
                FSM.transition_state("FINISHED")
            
        elif FSM.state == "FINISHED":
            # Open the bridge and respawn objects
            rospy.loginfo("Finished mission...")
            
        rate.sleep()