#!/usr/bin/env python
import os
import rospy
import smbus
import numpy as np
from std_msgs.msg import Int32
from duckietown import DTROS
from dt_street_light_controller.msg import StreetLightController

class LED(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LED, self).__init__(node_name=node_name)
       
        #I2C Bus
        self.MyBus=smbus.SMBus(1)

        #parameters
        # Add the node parameters to the parameters dictionary and load their default values
        self.parameters['~red'] = None
        self.parameters['~green'] = None
        self.parameters['~blue'] = None
        self.updateParameters()


        #parameters
        #grb instead of rgb
        self.color = [self.parameters['~green'],self.parameters['~red'],self.parameters['~blue']]
        self.msg_StreetLightController_array = np.zeros(4)
        self.log (self.color)

        # set the light to the values used for providing optimal lighting conditions
        for i in range (16):
            #green
            self.MyBus.write_byte_data(0x40,3*i,self.color[0])
            #red
            self.MyBus.write_byte_data(0x40,3*i+1, self.color[1])
            #blue
            self.MyBus.write_byte_data(0x40,3*i+2, self.color[2])

        #construct Subscriber
        self.sub = rospy.Subscriber("StreetLightController",StreetLightController, self.cbStreetLightController,queue_size=1)

    def cbStreetLightController(self, data):
        # grb instead of rgb
        self.msg_StreetLightController_array[0] = data.g_control_input
        self.msg_StreetLightController_array[1] = data.r_control_input
        self.msg_StreetLightController_array[2] = data.b_control_input
        self.msg_StreetLightController_array[3] = data.lux_control_input


        self.log(data.g_control_input)
        self.log("r = %f, g = %f, b = % f" %(self.msg_StreetLightController_array[0],self.msg_StreetLightController_array[1],self.msg_StreetLightController_array[2]))

        #for the 3 different color: j=1 is green, j=2 is red, j=3 is blue
        #i is for the 16 LEDs
        for j in range (3):
            
            #at low value the light is set to zero
            if self.msg_StreetLightController_array[j] <= 0.1:
                for i in range (16): 
                    self.MyBus.write_byte_data(0x40,3*i+j,0)
            
            #at high value the light is set to maximum
            elif self.msg_StreetLightController_array[j] >= 0.9:
                for i in range (16):
                    self.MyBus.write_byte_data(0x40,3*i+j,self.color[j])

            #between the values are dependent on the message recieved by the controller
            else:
                for i in range (16):
                    self.MyBus.write_byte_data(0x40,3*i+j,int(self.color[j]*self.msg_StreetLightController_array[j]))
      
            

if __name__ == '__main__':
    # create the node
    node = LED(node_name='dt_led_controller_node')
    rospy.spin()
