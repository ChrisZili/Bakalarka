#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from dronekit import connect,VehicleMode, LocationGlobalRelative,LocationGlobal
from pymavlink import mavutil
import time
import math

class FlightController():
    def __init__(self,ipadress):
        rospy.init_node('flight_controller',anonymous=True)
        self.ipadress= ipadress
        self.max_speed = 1
        self.image_center  = (960,540) # x,y coordinate of center of image frame
        self.tag_corners= []
        self.tag_center= ()
        self.model_state_sub= rospy.Subscriber('/apriltag_ros_continuous_node/corners', Float64MultiArray,self.corners_detection)
        self.vehicle = self.vehicle_connect()
        


    def vehicle_connect(self):
        vehicle = connect(self.ipadress,wait_ready = True)
        vehicle.parameters['PLND_ENABLED']=1
        vehicle.parameters['PLND_TYPE'] = 1
        vehicle.parameters['PLND_EST_TYPE'] = 0
        vehicle.parameters['LAND_SPEED'] = 20
        rospy.loginfo("Connection established")

        return vehicle


    def arm_and_takeoff(self,target_height = 2):
        while self.vehicle.is_armable != True:
            time.sleep(0.1)
        rospy.loginfo("Vehicle is armable")

        self.vehicle.mode = VehicleMode('GUIDED')

        while self.vehicle.mode != 'GUIDED':
            time.sleep(0.1)
        rospy.loginfo("Vehicle is in mode GUIDED")

        self.vehicle.armed = True
        while self.vehicle.armed == False:
            rospy.loginfo("Waiting for vehicle to get armed")
            time.sleep(0.1)
        
        rospy.loginfo("Vehicle armed")
        
        self.vehicle.simple_takeoff(target_height)
        rospy.loginfo("Vehicle take off to target height : %d m",target_height)


        while True:
            rospy.loginfo("Current altitude : %d"%self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= 0.95*target_height:
                break
            time.sleep(0.1)
        
        rospy.loginfo("Vehicle reached target height : %d m",target_height)


    def flight_control(self) :

        while len(self.tag_corners) == 0:
            time.sleep(0.1)
            rospy.loginfo("Finding tags")
        
        rospy.loginfo("CENTER : x = %s, y = %s",self.tag_center[0],self.tag_center[1])

        object_center_pos_x = self.tag_center[0] - self.image_center[0]
        object_center_pos_y = self.image_center[1] - self.tag_center[1]

        angle_offset_local = self.get_angle_offset(cx=object_center_pos_x,cy=object_center_pos_y)
        angle_offset = angle_offset_local + self.vehicle.heading
        
        norm_koef = math.sqrt(math.pow(self.image_center[0],2)) + math.sqrt(math.pow(self.image_center[1],2))

        velocity_x = math.cos(math.radians(angle_offset)) * \
                    self.max_speed * (self.dist_between_points(0,0,object_center_pos_x,object_center_pos_y) / norm_koef)
        velocity_y = math.sin(math.radians(angle_offset)) * \
                    self.max_speed * (self.dist_between_points(0,0,object_center_pos_x,object_center_pos_y) / norm_koef)
        velocity_z = 0

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, # time_boot_ms
            0,0, # target system,target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,#frame
            0b0000111111000111, # type mask(only speeds enabled)
            0, 0, 0, # x,y,z positions
            velocity_x, # x velocity in m/s
            velocity_y, # y velocity in m/s
            velocity_z, # z velocity in m/s
            0,0,0,# x, y, z acceleration(not supported yet, ignored in GCS_Mavlink)
            0,0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def get_angle_offset(self,cx,cy):
        phi = 0
        if((cx > 0 and cy > 0) and cx != 0): 
            phi = math.degrees(math.atan(float(cx) / float(cy)))
        if((cx > 0 and cy < 0) and cx != 0):
            phi = 90 + math.degrees(math.atan(float(-cy) / float(cx)))
        if((cx < 0 and cy < 0) and cx != 0):  
            phi =90 + 90 + math.degrees(math.atan(float(-cx) / float(-cy)))
        if((cx < 0 and cy > 0) and cx != 0):  
            phi = 90+90 + 90 + math.degrees(math.atan(float(cy) / float(-cx)))
        if(cx == 0 and cy > 0):
            phi = 0
        if(cx == 0 and cy < 0):
            phi = 180

        return phi


    def dist_between_points(self,x1, y1, x2, y2) :
        dx = x2 - x1
        dy = y2 - y1
        dist = math.sqrt(dx*dx + dy*dy)
        return dist
    
    def corners_detection(self,msg):
        
        self.tag_corners = self.process_corners(msg.data)
        self.tag_center = self.get_tag_center(self.tag_corners)
        
        # rospy.loginfo("LEFT BOTTOM CORNER : x = %s, y = %s",self.tag_corners[0][0],self.tag_corners[0][1])
        # rospy.loginfo("RIGHT BOTTOM CORNER : x = %s, y = %s",self.tag_corners[1][0],self.tag_corners[1][1])
        # rospy.loginfo("RIGHT TOP CORNER : x = %s, y = %s",self.tag_corners[2][0],self.tag_corners[2][1])
        # rospy.loginfo("LEFT TOP CORNER : x = %s, y = %s",self.tag_corners[3][0],self.tag_corners[3][1])
        # rospy.loginfo("CENTER : x = %s, y = %s",self.tag_center[0],self.tag_center[1])

        
    def process_corners(self,data):
        corners = []
       
        corners.append([data[0],data[1]])
        corners.append([data[2],data[3]])
        corners.append([data[4],data[5]])
        corners.append([data[6],data[7]])

        return corners


    def get_tag_center(self,corners):
        distance_x = (abs(corners[1][0] - corners[0][0])/2)
        distance_y = (abs(corners[2][1] - corners[1][1])/2)
        x = int((corners[0][0]) + distance_x)
        y = int((corners[2][1]) + distance_y)
     
        return (x,y)

            
    def run(self):

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.flight_control()
            rate.sleep()

if __name__ == '__main__':
    
    flight_controlller = FlightController('udp:127.0.0.1:14551') 
    while 1:
        command = raw_input("COMMAND : ")
        if command.upper() == 'TAKEOFF':
            flight_controlller.arm_and_takeoff(2)
            flight_controlller.run()


        elif command.upper() == 'FOLLOW':
            flight_controlller.run()
        