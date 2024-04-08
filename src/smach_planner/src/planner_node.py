#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from pcl_utils.srv import GetPose
import actionlib
from actionlib_msgs.msg import GoalID

# Global variables
global action_client
global_box_num = ""
global_target_pose = None
box_selected = False  # Flag to indicate if a box has been selected
preemption_requested = False
cancel_preemption_requested = False

# Helper function to get room pose
def get_initial_pose():
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = 15.5
    pose.pose.position.y = -6.45
    pose.pose.orientation.z = 0.7182659234184829 
    pose.pose.orientation.w = 0.6957686851646846 
    return pose

def cancel_goal_cb(msg):
    global cancel_preemption_requested
    # Set the flag to true whenever any message is received on this topic
    cancel_preemption_requested = True
    rospy.loginfo("Cancel goal requested.")
      
# Initialize the action client
action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

def rviz_panel_cb(msg):
    global global_box_num, box_selected, global_target_pose, preemption_requested
    # Trigger preemption for specific button presses
    if msg.data in ["/vehicle_1", "/vehicle_2", "/vehicle_3", "/assembly_line_1", "/assembly_line_2"]:
        preemption_requested = True
        box_selected = False
        global_box_num = ""
        rospy.loginfo(f"Preemption requested due to {msg.data} button press.")
    elif msg.data.startswith("/box_"):
        # Normal box selection logic here
        new_box_num = "box_number: " + msg.data.split("_")[-1]
        if new_box_num != global_box_num:
            global_box_num = new_box_num
            box_selected = True
            rospy.loginfo(f"New box selected: {global_box_num}")
            global_target_pose = get_initial_pose()
        else:
            rospy.loginfo(f"Box {global_box_num} re-selected.")
            
class Preempted(smach.State):
    def __init__(self):
        # Only an 'idle' outcome that leads back to the IDLE state
        smach.State.__init__(self, outcomes=['idle'])

    def execute(self, userdata):
        global box_selected, global_box_num
        rospy.loginfo("Preempted! Resetting box selection state.")
        box_selected = False
        global_box_num = ""
        return 'idle'

# State 0: Idle
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['box_selected', 'idle'],
                             output_keys=['target_pose', 'box_num', 'pose_adjustment_counter'],
                             input_keys=['box_selected', 'target_pose'])
        
    def execute(self, userdata):
        global box_selected, global_box_num, global_target_pose
        rospy.sleep(1)  # Sleep to allow for topic processing
        if box_selected:
            userdata.target_pose = global_target_pose
            userdata.box_num = global_box_num
            userdata.pose_adjustment_counter = 0
            return 'box_selected'
        return 'idle'

# State 1: for navigating to point
class NavigateToPoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['target_pose'],
                             output_keys =['target_pose'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        global box_selected, global_box_num, preemption_requested, cancel_preemption_requested
        if preemption_requested:
            preemption_requested = False  # Reset for next action
            return 'preempted'  # Use the outcome that transitions to IDLE
        if cancel_preemption_requested:
            cancel_preemption_requested = False  # Reset the flag
            return 'preempted'
        if not box_selected:
            return 'preempted'
        
        rospy.loginfo(f"NavigateToPoint state entered with {global_box_num}")
        goal = MoveBaseGoal()
        goal.target_pose = userdata['target_pose']
        self.client.send_goal(goal)
        
        while not rospy.is_shutdown():
            if self.client.wait_for_result(rospy.Duration(0.5)):
                state = self.client.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    return 'succeeded'
                else:
                    return 'aborted'
            if not box_selected:
                self.client.cancel_goal()
                return 'preempted'

# State 1b: adjusting the goal pose
class AdjustingInitialPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['adjusted','preempted'],
                             input_keys=['pose_adjustment_counter', 'target_pose'],
                             output_keys=['pose_adjustment_counter', 'target_pose'])

    def execute(self, userdata):
        global preemption_requested, cancel_preemption_requested
        if preemption_requested:
            preemption_requested = False  # Reset for next action
            return 'preempted'  # Use the outcome that transitions to IDLE
        if cancel_preemption_requested:
            cancel_preemption_requested = False  # Reset the flag
            return 'preempted'
        
        rospy.loginfo("Adjusting initial pose due to aborted navigation.")
        # Adjust the target_pose by moving 1.0m to the y position
        userdata.target_pose.pose.position.x -= 1.0
        userdata.pose_adjustment_counter += 1
        #counter adjustment, to use to set the number of iteration)
        return 'adjusted'

# State 2: ReachedInititalPose
class ReachedInitialPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'preempted'],
                             output_keys=['turn_counter', 'move_counter', 'fail_counter', 'minor_counter', 'minor_adj'])

    def execute(self, userdata):
        global preemption_requested, cancel_preemption_requested
        if preemption_requested:
            preemption_requested = False  # Reset for next action
            return 'preempted'  # Use the outcome that transitions to IDLE
        if cancel_preemption_requested:
            cancel_preemption_requested = False  # Reset the flag
            return 'preempted'
        
        rospy.loginfo('Reached initial pose. Initializing counters.')
        userdata.turn_counter = 0
        userdata.move_counter = 0
        userdata.fail_counter = 0
        userdata.minor_counter = 0
        userdata.minor_adj = False
        return 'done'

# State 3: Start recognition
class StartRecognition(smach.State):
    def __init__(self):
        # Include 'retry_recognition' in the list of outcomes
        smach.State.__init__(self, outcomes=['detection_process', 'retry_recognition', 'minor_adjustment', 'preempted'],
                             input_keys=['turn_counter', 'move_counter', 'minor_counter'],
                             output_keys=['turn_counter', 'move_counter', 'minor_counter', 'detected_pose', 'minor_adj', 'detect_code']) #box_selected
        self.recognition_service = rospy.ServiceProxy('get_pose', GetPose)

    def execute(self, userdata):
        global global_box_num, preemption_requested, cancel_preemption_requested
        if preemption_requested:
            preemption_requested = False  # Reset for next action
            return 'preempted'  # Use the outcome that transitions to IDLE
        if cancel_preemption_requested:
            cancel_preemption_requested = False  # Reset the flag
            return 'preempted'
        
        rospy.loginfo('Initiating recognition procedure for {}'.format(global_box_num))
        box_number = int(global_box_num.split(": ")[1])
        
        try:
            response = self.recognition_service(box_number=box_number)
            rospy.loginfo('Recognition service response: success_code={}'.format(response.success_code))
            
            # Decide the outcome based on the response's success_code
            userdata.detect_code = response.success_code
            if response.success_code == 0:
                rospy.loginfo("Recognition success.")
                userdata.detected_pose = response.coor_wrt_map
                # global box_selected  # Reference the global variable
                # global_box_num = ""
                # box_selected = False    # Reset box selection flag
                return 'detection_process'
            elif response.success_code in [2, 3, 4]:
                userdata.minor_adj = True
                userdata.minor_counter += 1
                return 'minor_adjustment'
            else:
                userdata.minor_adj = False
                userdata.minor_counter = 0
                # If recognition is not successful, retry recognition
                return 'retry_recognition'
                
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            # If the service call fails, also attempt to retry recognition
            return 'retry_recognition'
 
 # State 3B: Adjustment For Recognition
class AdjustPositionForRecognition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['retry_recognition', 'retry_adjustment', 'failed', 'preempted'],
                             input_keys=['turn_counter', 'move_counter', 'target_pose', 'pose_adjustment_counter'],
                             output_keys=['turn_counter', 'move_counter', 'target_pose'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.skip_requested = False
    
    def skipahead_cb(self, msg):
        self.skip_requested = True
        rospy.loginfo("Skip ahead requested.")

    def execute(self, userdata):
        self.skip_subscriber = rospy.Subscriber("/skip_ahead", String, self.skipahead_cb)
        global preemption_requested, cancel_preemption_requested
        if preemption_requested:
            preemption_requested = False  # Reset for next action
            self.skip_subscriber.unregister()
            return 'preempted'  # Use the outcome that transitions to IDLE
        if cancel_preemption_requested:
            cancel_preemption_requested = False  # Reset the flag
            self.skip_subscriber.unregister()
            return 'preempted'
        
        rospy.loginfo('Adjusting position for another recognition attempt.')
        # Adjust position based on turn_counter and move_counter
        if userdata.turn_counter == 0:
            move_threshold = 8 - userdata.pose_adjustment_counter
        elif userdata.turn_counter == 1:
            move_threshold = 8
        elif userdata.turn_counter == 2:
            move_threshold = 10
        elif userdata.turn_counter ==3:
            move_threshold = 6
        
        need_to_move = False
        
        # Check if it's time to rotate or terminate
        if userdata.move_counter >= move_threshold:
            if userdata.turn_counter == 0:
                new_position = {'x': 7.0, 'y': -6.55}  
                new_orientation = {'z': 0.0036402913900478438, 'w': 0.9999933741173466}
                userdata.target_pose.pose.position.x = new_position['x']
                userdata.target_pose.pose.position.y = new_position['y']
                userdata.target_pose.pose.orientation.z = new_orientation['z']
                userdata.target_pose.pose.orientation.w = new_orientation['w']
                rospy.loginfo(f"Rotation performed due to turn_counter = 0")
                need_to_move = True
            elif userdata.turn_counter == 1:
                new_position = {'x': 7.0, 'y': 1.75}
                new_orientation = {'z': -0.7076704123075005, 'w': 0.7065427004396353}
                userdata.target_pose.pose.position.x = new_position['x']
                userdata.target_pose.pose.position.y = new_position['y']
                userdata.target_pose.pose.orientation.z = new_orientation['z']
                userdata.target_pose.pose.orientation.w = new_orientation['w']
                rospy.loginfo(f"Rotation performed due to turn_counter = 1")
                need_to_move = True
            elif userdata.turn_counter == 2:
                new_position = {'x': 17.0, 'y': 1.25}
                new_orientation = {'z': 0.9972082120865974, 'w': 0.0746711573972971}
                userdata.target_pose.pose.position.x = new_position['x']
                userdata.target_pose.pose.position.y = new_position['y']
                userdata.target_pose.pose.orientation.z = new_orientation['z']
                userdata.target_pose.pose.orientation.w = new_orientation['w']
                rospy.loginfo("Rotation performed due to turn_counter = 2")
                need_to_move = True
            elif userdata.turn_counter == 3:
                rospy.loginfo("Terminating after full rotation.")
                self.skip_subscriber.unregister()
                return 'failed'
            
            userdata.turn_counter += 1
            userdata.move_counter = 0
            self.skip_subscriber.unregister()
            return 'retry_recognition'
        
        else:
        # Adjust position for next recognition attempt
            if userdata.turn_counter == 0:
                userdata.target_pose.pose.position.x -= 1.0  # Move in -x direction
            elif userdata.turn_counter == 1: 
                userdata.target_pose.pose.position.y += 1.0  # Move in +y direction
            elif userdata.turn_counter == 2:
                userdata.target_pose.pose.position.x += 1.0  # Move in +x direction
            elif userdata.turn_counter == 3:
                userdata.target_pose.pose.position.y -= 1.0  # Move in -y direction
            
            need_to_move = True
            userdata.move_counter += 1
        
        if need_to_move:
            if self.skip_requested:
                rospy.loginfo("Skip requested, proceeding to retry adjustment.")
                self.skip_requested = False  # Reset the skip request flag
                self.skip_subscriber.unregister()
                return 'retry_adjustment'
            
            # After adjusting target_pose, send a new goal to move_base
            new_goal = MoveBaseGoal()
            new_goal.target_pose.header.frame_id = "map"
            new_goal.target_pose.header.stamp = rospy.Time.now()
            new_goal.target_pose.pose = userdata['target_pose'].pose
        
            # Send the new goal to move_base
            self.client.send_goal(new_goal)
            
            while not rospy.is_shutdown():
                if self.skip_requested:
                    rospy.loginfo("Skip requested, cancelling current goal and proceeding to retry adjustment.")
                    self.skip_requested = False  # Reset the skip request flag
                    self.skip_subscriber.unregister()
                    return 'retry_adjustment'

                if self.client.wait_for_result(rospy.Duration(0.5)):  # Check every 0.5 seconds
                    action_state = self.client.get_state()
                    if action_state == actionlib.GoalStatus.SUCCEEDED:
                        rospy.loginfo("Robot moved to the new position successfully.")
                        self.skip_subscriber.unregister()
                        return 'retry_recognition'
                    elif action_state == actionlib.GoalStatus.ABORTED:
                        rospy.loginfo("Obstacle detected on the spot.")
                        self.skip_requested = False 
                        self.skip_subscriber.unregister()
                        return 'retry_adjustment'
                    else:
                        rospy.loginfo("Failed to move the robot to the new position.")
                        self.skip_subscriber.unregister()
                        self.skip_requested = False 
                        return 'retry_adjustment'
            
        
        # If moving was not necessary but skip was requested during state execution
        if self.skip_requested:
            rospy.loginfo("Skip requested, but no navigation was needed. Proceeding to retry adjustment.")
            self.skip_requested = False
            self.skip_subscriber.unregister()
            return 'retry_adjustment'

 # State 3C: Minor Adjustment For Recognition            
class MinorAdjustment(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['retry_recognition','retry_adjustment','preempted'],
                             input_keys=['target_pose', 'minor_counter', 'turn_counter', 'minor_pose', 'detect_code'],
                             output_keys=['target_pose', 'minor_counter', 'turn_counter', 'minor_pose','detect_code'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.skip_requested = False
            
    def skipahead_cb(self, msg):
        self.skip_requested = True
        rospy.loginfo("Skip ahead requested.")
        
    def execute(self, userdata):    
        self.skip_subscriber = rospy.Subscriber("/skip_ahead", String, self.skipahead_cb)
        global preemption_requested, cancel_preemption_requested
        if preemption_requested:
            preemption_requested = False  # Reset for next action
            self.skip_subscriber.unregister()
            return 'preempted'  # Use the outcome that transitions to IDLE
        if cancel_preemption_requested:
            cancel_preemption_requested = False  # Reset the flag
            self.skip_subscriber.unregister()
            return 'preempted'
    
        rospy.loginfo("Minor adjustment before retrying recognition.")
        
        if userdata.minor_counter <= 1:
            userdata.minor_pose = userdata.target_pose
         
        if userdata.turn_counter == 0:
            if userdata.detect_code == 2: #left side
                userdata.minor_pose.pose.position.x -= 0.5
                
            elif userdata.detect_code == 4:#right side
                userdata.minor_pose.pose.position.x += 0.5
                
            elif userdata.detect_code == 3: #middle
                userdata.minor_pose.pose.position.y += 0.5
                
        elif userdata.turn_counter == 1:
            if userdata.detect_code == 2: #left side
                userdata.minor_pose.pose.position.y += 0.5
                
            elif userdata.detect_code == 4:#right side
                userdata.minor_pose.pose.position.y -= 0.5
                
            elif userdata.detect_code == 3: #middle
                userdata.minor_pose.pose.position.x += 0.5
                
        elif userdata.turn_counter == 2:
            if userdata.detect_code == 2: #left side
                userdata.minor_pose.pose.position.x += 0.5
                
            elif userdata.detect_code == 4:#right side
                userdata.minor_pose.pose.position.x -= 0.5
                
            elif userdata.detect_code == 3: #middle
                userdata.minor_pose.pose.position.y -= 0.5
                
        elif userdata.turn_counter == 3:
            if userdata.detect_code == 2: #left side
                userdata.minor_pose.pose.position.y -= 0.5
                
            elif userdata.detect_code == 4:#right side
                userdata.minor_pose.pose.position.y += 0.5
                
            elif userdata.detect_code == 3: #middle
                userdata.minor_pose.pose.position.x -= 0.5
        
        if self.skip_requested:
            rospy.loginfo("Skip requested, proceeding to retry adjustment.")
            self.skip_requested = False  # Reset the skip request flag
            self.skip_subscriber.unregister()
            return 'retry_adjustment'
        
        # After adjusting target_pose, send a new goal to move_base
        new_goal = MoveBaseGoal()
        new_goal.target_pose.header.frame_id = "map"
        new_goal.target_pose.header.stamp = rospy.Time.now()
        new_goal.target_pose.pose = userdata['minor_pose'].pose
        
        # Send the new goal to move_base
        self.client.send_goal(new_goal)
        
            
        while not rospy.is_shutdown():
            if self.skip_requested:
                rospy.loginfo("Skip requested, cancelling current goal and proceeding to retry adjustment.")
                self.skip_requested = False  # Reset the skip request flag
                self.skip_subscriber.unregister()
                return 'retry_adjustment'

            if self.client.wait_for_result(rospy.Duration(0.5)):  # Check every 0.5 seconds
                action_state = self.client.get_state()
                if action_state == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Robot moved to the new position successfully.")
                    self.skip_subscriber.unregister()
                    return 'retry_recognition'
                elif action_state == actionlib.GoalStatus.ABORTED:
                    rospy.loginfo("Obstacle detected on the spot.")
                    self.skip_requested = False 
                    self.skip_subscriber.unregister()
                    return 'retry_adjustment'
                else:
                    rospy.loginfo("Failed to move the robot to the new position.")
                    self.skip_subscriber.unregister()
                    self.skip_requested = False 
                    return 'retry_adjustment'

 # State 4: Evaluate Detection   
class Detection_Process(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'retry_recognition', 'minor_adjustment','failed'],
                             input_keys=['detected_pose', 'target_pose', 'turn_counter', 'minor_adj', 'minor_pose', 'minor_counter', 'detect_code'],
                             output_keys=['minor_counter', 'minor_pose','detect_code', 'minor_adj'])
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        global preemption_requested, box_selected, global_box_num, cancel_preemption_requested
        if preemption_requested:
            preemption_requested = False  # Reset for next action
            return 'preempted'  # Use the outcome that transitions to IDLE
        if cancel_preemption_requested:
            cancel_preemption_requested = False  # Reset the flag
            return 'preempted'

        rospy.loginfo('Processing detected pose in Detection_Process')
        if userdata.minor_adj:
            robot_pose = userdata.minor_pose
        else:
            robot_pose = userdata.target_pose
            
        dx = robot_pose.pose.position.x - userdata.detected_pose.pose.position.x
        dy = robot_pose.pose.position.y - userdata.detected_pose.pose.position.y
        distance = math.sqrt(dx ** 2 + dy ** 2)
        rospy.loginfo(f"Distance to target: {distance} meters")
        
        distance_threshold = 2.0 # Define your distance threshold (meters)
        upper_distance_threshold = 5.0
        if distance >= distance_threshold:
            if distance >= upper_distance_threshold:
                rospy.loginfo("Detection deemed invalid due to distance. Retrying recognition...")
                return 'retry_recognition'
            else: 
                rospy.loginfo("Detection deemed invalid due to distance. Going into Minor Adjustment ...")
                userdata.minor_pose = robot_pose
                userdata.minor_counter = 2
                userdata.minor_adj = True
                userdata.detect_code = 3
                return 'minor_adjustment'
        
        else:
            rospy.loginfo("Detection valid.")
            
            if userdata.turn_counter == 1:
                new_orientation_z = 0.0036402913900478438
                new_orientation_w = 0.9999933741173466
                userdata.detected_pose.pose.position.y -= 0.2
            elif userdata.turn_counter == 2:
                new_orientation_z = -0.7076704123075005
                new_orientation_w =  0.7065427004396353
                userdata.detected_pose.pose.position.x -= 0.2
            elif userdata.turn_counter == 3:
                new_orientation_z = -0.9999988691415771
                new_orientation_w = 0.0015039001186830257
                userdata.detected_pose.pose.position.y += 0.2
            elif userdata.turn_counter == 0:
                new_orientation_z = 0.7182659234184829
                new_orientation_w = 0.6957686851646846
                userdata.detected_pose.pose.position.x += 0.2
            else:
                # Default orientation or another logic for different turn_counter values
                new_orientation_z = userdata.detected_pose.pose.orientation.z
                new_orientation_w = userdata.detected_pose.pose.orientation.w
            
            userdata.detected_pose.pose.orientation.z = new_orientation_z
            userdata.detected_pose.pose.orientation.w = new_orientation_w
            detected_pose = userdata.detected_pose
            
            goal = MoveBaseGoal()
            goal.target_pose = detected_pose
            self.client.send_goal(goal)
        
            # Wait for the result
            self.client.wait_for_result()
            # Check the result
            action_state = self.client.get_state()
            if action_state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Success navigating to the detected goal.")
                global_box_num = ""
                box_selected = False  # Reset box selection flag
                return 'success'
            else:
                rospy.loginfo("Failed to navigate to the detected goal.")
                global_box_num = ""
                box_selected = False  # Reset box selection flag
                return 'failed'

# State 5: Success
class Success(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted'])

    def execute(self, userdata):
        global preemption_requested, cancel_preemption_requested
        if preemption_requested:
            preemption_requested = False  # Reset for next action
            return 'preempted'  # Use the outcome that transitions to IDLE
        if cancel_preemption_requested:
            cancel_preemption_requested = False  # Reset the flag
            return 'preempted'
        
        rospy.loginfo('Reached the target box! Task Completed.')
        return 'preempted'

def main():
    global action_client
    rospy.init_node('smach_planner_node')
    
    rospy.Subscriber("/rviz_panel/goal_name", String, rviz_panel_cb)
    rospy.Subscriber("/move_base/cancel", GoalID, cancel_goal_cb)
    
    
    action_client.wait_for_server()
    
    # Ensure the recognition service is available before proceeding
    rospy.loginfo("Waiting for 'get_pose' service...")
    rospy.wait_for_service('get_pose')
    rospy.loginfo("'get_pose' service available.")

    sm = smach.StateMachine(outcomes=['end', 'failed'])
    with sm:
        # State: Idle
        smach.StateMachine.add('IDLE', Idle(), 
                               transitions={'box_selected':'NAVIGATE_TO_POINT', 
                                            'idle':'IDLE'},
                               remapping={'target_pose':'target_pose', 
                                          'box_num':'box_num',
                                          'pose_adjustment_counter': 'pose_adjustment_counter'})
        
        smach.StateMachine.add('PREEMPTED', Preempted(), 
                           transitions={'idle': 'IDLE'})

        # State: NavigateToPoint
        smach.StateMachine.add('NAVIGATE_TO_POINT', NavigateToPoint(), 
                               transitions={'succeeded':'REACHED_INITIAL_POSE',
                                            'preempted':'PREEMPTED',
                                            'aborted':'ADJUSTING_INITIAL_POSE'},
                               remapping={'target_pose':'target_pose',
                                          'pose_adjustment_counter':'pose_adjustment_counter'}) 

        
        # State: AdjustingInitialPose
        smach.StateMachine.add('ADJUSTING_INITIAL_POSE', AdjustingInitialPose(),
                               transitions={'preempted':'PREEMPTED',
                                            'adjusted':'NAVIGATE_TO_POINT'},
                                remapping={'target_pose':'target_pose',
                                           'pose_adjustment_counter':'pose_adjustment_counter'})  # Go back to NavigateToPoint after adjustment
        
        # State: ReachedInitialPose
        smach.StateMachine.add('REACHED_INITIAL_POSE', ReachedInitialPose(), 
                               transitions={'preempted': 'PREEMPTED',
                                            'done':'START_RECOGNITION'},
                               remapping={'turn_counter':'turn_counter', 
                                          'move_counter':'move_counter',
                                          'fail_counter':'fail_counter',
                                          'minor_counter':'minor_counter',
                                          'minor_adj':'minor_adj',
                                          'pose_adjustment_counter':'pose_adjustment_counter',
                                          'target_pose':'target_pose'})
        
        # State to initiate the recognition procedure
        smach.StateMachine.add('START_RECOGNITION', StartRecognition(),
                               transitions={'preempted': 'PREEMPTED',
                                            'detection_process':'DETECTION_PROCESS',
                                            'minor_adjustment':'MINOR_ADJUSTMENT', 
                                            'retry_recognition':'ADJUST_POSITION_FOR_RECOGNITION'},
                               remapping={'turn_counter':'turn_counter', 
                                          'move_counter':'move_counter',
                                          'minor_counter':'minor_counter',
                                          'minor_adj':'minor_adj',
                                          'pose_adjustment_counter':'pose_adjustment_counter',
                                          'detected_pose':'detected_pose',
                                          'detect_code':'detect_code',
                                          'target_pose':'target_pose',
                                          'minor_pose':'minor_pose'})
        
        # State for adjusting position and retrying recognition
        smach.StateMachine.add('ADJUST_POSITION_FOR_RECOGNITION', AdjustPositionForRecognition(), 
                               transitions={'preempted': 'PREEMPTED',
                                            'retry_recognition':'START_RECOGNITION',
                                            'retry_adjustment':'ADJUST_POSITION_FOR_RECOGNITION', 
                                            'failed':'PREEMPTED'},
                               remapping={'turn_counter':'turn_counter', 
                                          'move_counter':'move_counter',
                                          'minor_counter':'minor_counter',
                                          'minor_adj':'minor_adj',
                                          'detect_code':'detect_code',
                                          'target_pose':'target_pose',
                                          'minor_pose':'minor_pose'})
        
        # State for minor adjustment
        smach.StateMachine.add('MINOR_ADJUSTMENT', MinorAdjustment(), 
                               transitions={'preempted': 'PREEMPTED',
                                            'retry_recognition':'START_RECOGNITION', 
                                            'retry_adjustment':'ADJUST_POSITION_FOR_RECOGNITION'},
                               remapping={'turn_counter':'turn_counter', 
                                          'move_counter':'move_counter',
                                          'minor_counter':'minor_counter',
                                          'minor_adj':'minor_adj',
                                          'detect_code':'detect_code',
                                          'target_pose':'target_pose',
                                          'minor_pose':'minor_pose'})
        
        # State for processing detection outcome
        smach.StateMachine.add('DETECTION_PROCESS', Detection_Process(), 
                       transitions={'success':'SUCCESS',
                                    'retry_recognition': 'ADJUST_POSITION_FOR_RECOGNITION',
                                    'minor_adjustment':'MINOR_ADJUSTMENT',
                                    'failed':'PREEMPTED'},
                       remapping={'detected_pose': 'detected_pose', 
                                  'target_pose': 'target_pose',
                                  'turn_counter': 'turn_counter',
                                  'minor_pose':'minor_pose',
                                  'minor_adj':'minor_adj',
                                  'minor_counter':'minor_counter',
                                  'detect_code':'detect_code',})
        
        # State: Success
        smach.StateMachine.add('SUCCESS', Success(), 
                               transitions={'preempted': 'PREEMPTED'})

        
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    
    rospy.spin()
    sis.stop()
    
    if outcome == 'failed':
        rospy.logerr("State machine ended with failure.")
    else:
        rospy.loginfo("State machine completed successfully.")
    
if __name__ == '__main__':
    main()
