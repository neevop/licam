#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import PoseStamped
from roborts_msgs.msg import GlobalPlannerActionResult
from move_base_msgs.msg import MoveBaseActionResult
'''
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
actionlib_msgs/GoalStatus status
  uint8 PENDING=0
  uint8 ACTIVE=1
  uint8 PREEMPTED=2
  uint8 SUCCEEDED=3
  uint8 ABORTED=4
  uint8 REJECTED=5
  uint8 PREEMPTING=6
  uint8 RECALLING=7
  uint8 RECALLED=8
  uint8 LOST=9
  actionlib_msgs/GoalID goal_id
    time stamp
    string id
  uint8 status
  string text
roborts_msgs/GlobalPlannerResult result
  int32 error_code
'''

'''
rz  -> x y z      w
0   -> 0 0 0      1
90  -> 0 0 0.707  0.707
180 -> 0 0 1      0
-90 -> 0 0 -0.707 0.707
'''

class FerryMode():
    def __init__(self):
        
        self.result_topic = "/move_base/result"
        self.goal_num = 0
        self.goal0_msg = PoseStamped()
        self.goal1_msg = PoseStamped()
        
        # self.goal0_msg.header.stamp = rospy.Time.now()
        self.goal0_msg.header.frame_id = 'map'
        
        self.goal0_msg.pose.position.x = 4.5
        self.goal0_msg.pose.position.y = 5.0
        self.goal0_msg.pose.position.z = 0.0
        
        self.goal0_msg.pose.orientation.x = 0.0
        self.goal0_msg.pose.orientation.y = 0.0
        self.goal0_msg.pose.orientation.z = -0.707
        self.goal0_msg.pose.orientation.w = 0.707
        
        # self.goal1_msg.header.stamp = rospy.Time.now()
        self.goal1_msg.header.frame_id = 'map'
        
        self.goal1_msg.pose.position.x = 5.5
        self.goal1_msg.pose.position.y = -5.0
        self.goal1_msg.pose.position.z = 0.0
        
        self.goal1_msg.pose.orientation.x = 0.0
        self.goal1_msg.pose.orientation.y = 0.0
        self.goal1_msg.pose.orientation.z = 0.707
        self.goal1_msg.pose.orientation.w = 0.707
        
        
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
    def callback(self, data):
        if data.status.status == 3:
            if self.goal_num == 0:
                print("PUBLISH goal0!!!!")
                self.goal0_msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.goal0_msg)
                self.goal_num = 1
            elif self.goal_num == 1:
                print("PUBLISH goal1!!!!")
                self.goal1_msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.goal1_msg)
                self.goal_num = 0
            else:
                raise Exception("Invalid goal_num!", self.goal_num)
        elif data.status.status == 4:
            if self.goal_num == 0:
                print("RE-PUBLISH goal1!!!!")
                self.goal1_msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.goal1_msg)
            elif self.goal_num == 1:
                print("RE-PUBLISH goal0!!!!")
                self.goal0_msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.goal0_msg)
            else:
                raise Exception("Invalid goal_num!", self.goal_num)
        elif data.status.status == 2:
            if self.goal_num == 0:
                print("GOAL 1 has been PREEMPTED!!!")
                self.goal_num = 1
            elif self.goal_num == 1:
                print("GOAL 0 has been PREEMPTED!!!")
                self.goal_num = 0
            else:
                raise Exception("Invalid goal_num!", self.goal_num)
        else:
            raise Exception("Invalid status!", data.status.status)
        
    def node_run(self):
        rospy.init_node('ferry_mode_node', anonymous=True)
        rospy.Subscriber(self.result_topic, MoveBaseActionResult, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    ferry_mode_node = FerryMode()
    ferry_mode_node.node_run()
