#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState,ModelStates
from gazebo_msgs.srv import SetModelState,GetModelState
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler
import time
import math

class ModelController():
    def __init__(self, model_name):
        rospy.init_node('tag_controller',anonymous=True)
        self.current_state = ModelState()
        self.initial_state = self.init_state()
        self.model_name = model_name
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
    def model_state_callback(self,msg):
        # rospy.loginfo(msg)
        # rospy.loginfo(msg.name[1])
        pos = 0
        for inx in range(len(msg.name)):
            if msg.name[inx] == self.model_name:
                pos = inx
        self.current_state.model_name = msg.name[pos]
        self.current_state.pose = msg.pose[pos]
        self.current_state.twist = msg.twist[pos]


    def init_state(self):
        state = ModelState()
        state.model_name= 'april_tag'
        
        state.pose.position.x = 1.0
        state.pose.position.y = 0.0
        state.pose.position.z = 0.3

        state.pose.orientation.x = 0.0
        state.pose.orientation.y = 0
        state.pose.orientation.z = 0
        state.pose.orientation.w = 0.0




        state.twist.linear.x = 0
        state.twist.linear.y = 0
        state.twist.linear.z = 0

        state.twist.angular.x = 0
        state.twist.angular.y = 0
        state.twist.angular.z = 0

        state.reference_frame = 'world'
        rospy.loginfo(state)

        return state



    def create_vector(start, end, num):
        step = (end - start) / (num - 1)

        # Create the vector
        vector = [start + i * step for i in range(num)]

        return vector
    def turn_90_interval(self,desired_angle,velocity = 2.0):
        state = ModelState()
        twist = Twist()

        turn_time = abs(desired_angle/velocity)
        steps = (self.current_state.pose.orientation.z,self.current_state.pose.orientation.z+desired_angle,turn_time)

        state.model_name= self.model_name
        state.pose.position = self.current_state.pose.position
        state.pose.orientation = self.current_state.pose.orientation
        state.twist.linear = self.current_state.twist.linear
        state.twist.angular = self.current_state.twist.angular


        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0

        if(desired_angle < 0):
            velocity = -velocity
        twist.angular.z = velocity
        state.twist = twist
        self.set_model_state(state)

        time.sleep(turn_time)
        state.pose.position = self.current_state.pose.position
        state.pose.orientation = self.current_state.pose.orientation
        state.twist.linear = self.current_state.twist.linear
        state.twist.angular = self.current_state.twist.angular
        state.twist.angular.z = 0
        self.set_model_state(state)

    
        
    def set_model_state_velocity(self, vx=0, vy=0, vz=0, ax=0, ay=0, az=0):
        
        state = ModelState()
        state.model_name= self.model_name
        state.pose.position = self.current_state.pose.position
        state.pose.orientation = self.current_state.pose.orientation
        
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = vz
        twist.angular.x = ax
        twist.angular.y = ay
        twist.angular.z = az
        state.twist = twist
        state.reference_frame = 'world'
        print(state)

        self.set_model_state(state)
    
    def euler_from_quaternion(self,x, y, z, w):
    
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return yaw_z # in radians
    def current_yaw(self):
        return self.euler_from_quaternion(self.current_state.pose.orientation.x,self.current_state.pose.orientation.y,self.current_state.pose.orientation.z,self.current_state.pose.orientation.w)

    def move_north(self, velocity=0.3):
        rospy.loginfo(math.degrees(self.current_yaw()))
        self.turn_90_interval(desired_angle=-self.current_yaw())
        self.set_model_state_velocity(vx=velocity)
    
    def move_south(self, velocity=0.3):
        rospy.loginfo(math.degrees(self.current_yaw()))
        self.turn_90_interval(desired_angle=((math.pi))-self.current_yaw())

        self.set_model_state_velocity(vx=-velocity)
    
    def move_east(self, velocity=0.3):
        rospy.loginfo(math.degrees(self.current_yaw()))
        self.turn_90_interval(desired_angle=(-(math.pi/2))-self.current_yaw())
        self.set_model_state_velocity(vy=-velocity)
    
    def move_west(self, velocity=0.3):
        rospy.loginfo(math.degrees(self.current_yaw()))

        self.turn_90_interval(desired_angle=((math.pi/2))-self.current_yaw())
        self.set_model_state_velocity(vy=velocity)
    
    def move_north_east(self, velocity=0.3):
        rospy.loginfo(math.degrees(self.current_yaw()))

        self.turn_90_interval(desired_angle=(-(math.pi/4))-self.current_yaw())
        self.set_model_state_velocity(vx=velocity, vy=-velocity)
    
    def move_north_west(self, velocity=0.3):
        rospy.loginfo(math.degrees(self.current_yaw()))

        self.turn_90_interval(desired_angle=((math.pi/4))-self.current_yaw())
        self.set_model_state_velocity(vx=velocity, vy=velocity)
    
    def move_south_east(self, velocity=0.3):
        rospy.loginfo(math.degrees(self.current_yaw()))

        self.turn_90_interval(desired_angle=((5*math.pi)/4)-self.current_yaw())
        self.set_model_state_velocity(vx=-velocity, vy=-velocity)
    
    def move_south_west(self, velocity=0.3):
        rospy.loginfo(math.degrees(self.current_yaw()))

        self.turn_90_interval(desired_angle=((3*math.pi)/4)-self.current_yaw())
        self.set_model_state_velocity(vx=-velocity, vy=velocity)
    def stop(self):
        self.set_model_state_velocity(vx=0, vy=0,vz = 0)
    
    def trajectory(self):
        
        self.move_north()
        time.sleep(3)
        self.move_north_west()
        time.sleep(3)
        self.move_north()
        time.sleep(4)
        self.move_north_east()
        time.sleep(6)
        self.move_east()
        time.sleep(6)
        self.move_south_east()
        time.sleep(6)

        self.move_south()
        time.sleep(3)
        self.move_west()
        time.sleep(2)
        self.move_north_west()
        time.sleep(5)
        self.stop()



    def choose_direction(self):
    
        choice = raw_input('Direction>')
        print(choice)

        if choice.upper() == 'NORTH':
            model_controller.move_north()
        elif choice.upper() == 'SOUTH':
            model_controller.move_south()
        elif choice.upper() == 'EAST':
            model_controller.move_east()
        elif choice.upper() == 'WEST':
            model_controller.move_west()
        elif choice.upper() == 'NORTH-EAST':
            model_controller.move_north_east()
        elif choice.upper() == 'NORTH-WEST':
            model_controller.move_north_west()
        elif choice.upper() == 'SOUTH-EAST':
            model_controller.move_south_east()
        elif choice.upper() == 'SOUTH-WEST':
            model_controller.move_south_west()
        elif choice.upper() == 'STOP':
            model_controller.stop()
        elif choice.upper() == '90':
            model_controller.turn_90_interval()
        elif choice.upper() == 'STATE':
            rospy.loginfo(self.current_state)
        elif choice.upper() == 'RESET':
            self.set_model_state(self.initial_state)
        elif choice.upper() == 'START':
            self.trajectory()
        else:
            print('Invalid choice. Try again.')


        
    def main(self):
        # Loop at 10 Hz
        self.model_state_sub = rospy.Subscriber('/gazebo/model_states', ModelStates,self.model_state_callback)

        rate = rospy.Rate(10)

        # Main loop
        while not rospy.is_shutdown():
            # Process the latest model_states_msg here
            # ...
            self.choose_direction()
            
            # Sleep for the remainder of the loop
            rate.sleep()



if __name__ == '__main__':
    # Create an instance of the RobotControl class
    model_controller = ModelController('april_tag')

    # Call the main function
    model_controller.main()
