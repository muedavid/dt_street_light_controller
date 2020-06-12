#!/usr/bin/env python
import time
import RPi.GPIO as GPIO
import Adafruit_TCS34725
import smbus
import yaml
import os.path
import numpy as np
import rospy
from future.builtins import input
import shutil
#from duckietown_utils import get_duckiefleet_root
from Adafruit_GPIO import I2C
from duckietown import DTROS
from duckietown_msgs.msg import LightSensor
from dt_street_light_controller.msg import StreetLightController

class Controller(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(Controller, self).__init__(node_name=node_name)

        self.veh_name = rospy.get_namespace().strip("/")

        # GPIO setup
        # Choose BCM or BOARD numbering schemes
        # If LED wired to ground this isn't needed
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(18, GPIO.OUT)
        GPIO.output(18, GPIO.LOW)
        """



        #parameters
        self.parameters['~integration_time']=None
        self.parameters['~integration_gain']=None
        self.parameters['~k_I'] = None
        self.parameters['~k_p'] = None
        self.parameters['~integralsize']=None
        self.updateParameters()

        self.integration_time = self.parameters['~integration_time']
        self.integration_gain = self.parameters['~integration_gain']
        self.k_I= self.parameters['~k_I']
        self.k_p = self.parameters['~k_p']
        self.integralsize = self.parameters['~integralsize']
        self.r = 0
        self.g = 0
        self.b = 0
        self.lux = 0
        self.reference_lux = 0
        self.reference_red = 0
        self.reference_green = 0
        self.reference_blue = 0
        #PI controller
        self.integralsize = 20
        self.integralarray = np.zeros([self.integralsize,4])
        self.msg_array = np.zeros(4)
        self.integral = [0]*4
        self.antiwindup = [0]*4
        self.error = np.zeros([1,4])
        self.prev = [0]*4
        self.last_measurment = [0]*3
        self.off = 20
        
        
        #convert integration time and gain if form that it has to be
        self.convert_integration()
        
        #set integration time and value
        self.tcs = Adafruit_TCS34725.TCS34725(
            integration_time=self.integration_time,gain=self.integration_gain)
        
        self.tcs.set_interrupt_limits(0,2000)

        #Publisher
        self.msg = StreetLightController ()
        self.pub=rospy.Publisher("StreetLightController",StreetLightController,queue_size=1)

        #paths
        self.dirname = '/data/config/calibrations/street-light-controller/'
        self.filename = self.dirname  + self.veh_name + ".yaml"

        #read parameter of callibration
        self.readParamFromFile()
        rospy.sleep(2)
        
        
        #start Controller
        self.controller()

    def readParamFromFile(self):
        with open(self.filename, 'r') as in_file:
            yaml_dict = yaml.load(in_file)
            self.reference_lux= yaml_dict["lux"]
            self.reference_red= yaml_dict["r"]
            self.reference_green= yaml_dict["g"]
            self.reference_blue= yaml_dict["b"]



    def get_lux(self):
        # Read R, G, B, C color data from the sensor.
        try:
            self.r,self.g, self.b, self.c = self.tcs.get_raw_data()
        except:
            pass
        
        #avoid obviously wrong measurment
        if self.r >= 200:
            self.r = self.last_measurment[0]
        if self.g >= 200:
            self.g = self.last_measurment[1]
        if self.b >= 200: 
            self.b = self.last_measurment[2]
            
        self.last_measurment[0]=self.r
        self.last_measurment[1]=self.g
        self.last_measurment[2]=self.b

        self.msg.data_r=self.r
        self.msg.data_g=self.g
        self.msg.data_b=self.b
        # Calculate lux

        self.lux = Adafruit_TCS34725.calculate_lux(self.r, self.g, self.b)

    def convert_integration(self):
        #convert integration time in the form that it has to be
        if self.integration_time==700:
            self.integration_time = Adafruit_TCS34725.TCS34725_INTEGRATIONTIME_700MS
        if self.integration_time==154:
            self.integration_time = Adafruit_TCS34725.TCS34725_INTEGRATIONTIME_154MS
        if self.integration_time==101:
            self.integration_time = Adafruit_TCS34725.TCS34725_INTEGRATIONTIME_101MS
        if self.integration_time==50:
            self.integration_time = Adafruit_TCS34725.TCS34725_INTEGRATIONTIME_50MS
        if self.integration_time==24:
            self.integration_time = Adafruit_TCS34725.TCS34725_INTEGRATIONTIME_24MS
            
        #convert integration gain in the form that it has to be
        if self.integration_gain==1:
            self.integration_gain = Adafruit_TCS34725.TCS34725_GAIN_1X
        if self.integration_gain==4:
            self.integration_gain = Adafruit_TCS34725.TCS34725_GAIN_4X
        if self.integration_gain==16:
            self.integration_gain = Adafruit_TCS34725.TCS34725_GAIN_16X
        if self.integration_gain==60:
            self.integration_gain = Adafruit_TCS34725.TCS34725_GAIN_60X


    # hier den Controller programieren
    def controller(self):
        while not rospy.is_shutdown():
            
            self.get_lux()

            #normalized error
            self.error[0,0] = 1.0/float(self.reference_lux)*(self.reference_lux-self.lux)
            self.error[0,1] = 1.0/float(self.reference_red)*(self.reference_red-self.r)
            self.error[0,2] = 1.0/float(self.reference_green)*(self.reference_green-self.g)
            self.error[0,3] = 1.0/float(self.reference_blue)*(self.reference_blue-self.b)
            
            #only to record the rosbags and so validate the performance
            self.msg.error_r = self.error[0,1]
            self.msg.error_g = self.error[0,2]
            self.msg.error_b = self.error[0,3]
            
            for i in range (4): 
                self.integral[i] += self.error[0,i] - self.antiwindup[i]

                #input
                self.msg_array[i] = self.k_I*self.integral[i] + self.k_p*self.error[0,i]

                #saturation
                self.prev[i] = self.msg_array[i]
                if self.msg_array[i] <= 0:
                    self.msg_array[i] = 0
                elif self.msg_array[i] >= 1:
                    self.msg_array[i] = 1

                #antiwindup
                self.antiwindup[i] = self.prev[i] - self.msg_array[i]

            
            
            """
            #avoid having the light to much colored
            max = np.max([self.msg_array])
            for i in range (4):
                if (2*self.msg_array[i]) < max:
                    self.msg_array[i]= max / 2.0
            """

            #publisch message
            self.msg.lux_control_input = self.msg_array[0]
            self.msg.r_control_input = self.msg_array[1]
            self.msg.g_control_input = self.msg_array[2]
            self.msg.b_control_input = self.msg_array[3]
            
            self.pub.publish(self.msg)

            #print out the data for debug
            self.log ("error is")
            self.log (self.error)
            #self.log ("message is")
            #self.log (self.msg_array)
            self.log ("Data ist: lux = %d, r = %d, g = %d, b = %d" % (self.lux, self.r, self.g, self.b))
            """
            self.log ("integral values are")
            self.log (self.integral)
            self.log ("antiwindup")
            self.log (self.antiwindup)
            """



    def controller_with_alternative_saturation_solution(self):
        while not rospy.is_shutdown():
            
            self.get_lux()
            self.error[0,0] = 255.0/self.reference_lux*(self.reference_lux-self.lux)
            self.error[0,1] = 255.0/self.reference_red*(self.reference_lux-self.r)
            self.error[0,2] = 255.0/self.reference_green*(self.reference_lux-self.g)
            self.error[0,3] = 255.0/self.reference_blue*(self.reference_lux-self.b)
            
            #only to record the rosbags and so validate the performance
            self.msg.error_r = self.error[0,1]
            self.msg.error_g = self.error[0,2]
            self.msg.error_b = self.error[0,3]
            
            #sometimes if we have much light the sensor measure false value
            self.off +=1

            
            if (self.lux > self.lux_last[0] + 40) or self.lux < -20:
                self.off = 0
            

            #if self.off <5:
                #self.msg_array[0] = 0
                #self.msg_array[1] = 0
                #self.msg_array[2] = 0
                #self.msg_array[3] = 0
            
            else:

                # remove latest row of list
                self.integralarray = np.delete(self.integralarray, 0, axis=0)
                # add error as new row of the list
            
                self.integralarray = np.append(self.integralarray, self.error, axis=0)

                #we callibrate everything then the roomlight is off. If the roomlight is on self.error is strong negative. We don't want to have a to big integral
                for i in range (4):
                    if self.integralarray[self.integralsize-1,i] < -10:
                        self.integralarray[self.integralsize-1,i]

                #calculate the individual integrals
                self.integral = np.sum(self.integralarray, axis=1)


                for i in range (4):
                    # calculate the message that has to be published
                    self.msg_array[i] = self.k_I*self.integral[i] + self.k_p*self.error[0,i]+self.prev[i]

                    #prev can get realy large number and to limit this: (like a saturation that we set)
                    self.prev[i] = self.msg_array[i]
                    if self.prev[i] < -10:
                        self.prev[i] = -10
                    elif self.prev[i] > 260:
                        self.prev[i] = 260
            
                #publisch message

            #to not have the color to have to much only one color. 
            max = np.max([self.msg_array])
            for i in range (4):
                if (self.msg_array[i]+100) < max and (max-100) > 0:
                    self.msg_array[i]= max-100


            self.msg.lux_control_input = self.msg_array[0]
            self.msg.r_control_input = self.msg_array[1]
            self.msg.g_control_input = self.msg_array[2]
            self.msg.b_control_input = self.msg_array[3]
            
            self.pub.publish(self.msg)
            self.lux_last.pop(0)
            self.lux_last.append(self.lux)

            #print out the data for debug
            self.log("error is")
            self.log(self.error)
            self.log("message is")
            self.log (self.msg_array)
            self.log ("Data ist: lux = %d, r = %d, g = %d, b = %d" % (self.lux, self.r, self.g, self.b))
            


if __name__ == '__main__':
    # create the node
    node = Controller(node_name='dt_street_light_controller_node')
    rospy.spin()
