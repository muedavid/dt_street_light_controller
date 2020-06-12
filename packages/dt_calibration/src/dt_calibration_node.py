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
from Adafruit_GPIO import I2C
from duckietown import DTROS
from duckietown_msgs.msg import LightSensor
from dt_street_light_controller.msg import StreetLightController

class Calibration(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(Calibration, self).__init__(node_name=node_name)

        self.veh_name = rospy.get_namespace().strip("/")

        # GPIO setup
        # Choose BCM or BOARD numbering schemes
        #not necessary if wired LED to ground
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(18, GPIO.OUT)
        GPIO.output(18, GPIO.LOW)
        """



        #parameters
        self.parameters['~integration_time']=None
        self.parameters['~integration_gain']=None
        self.parameters['~num_calib'] = None
        self.updateParameters()

        self.integration_time = self.parameters['~integration_time']
        self.integration_gain = self.parameters['~integration_gain']
        self.num_calib = self.parameters['~num_calib']
        self.r = 0.0
        self.g = 0.0
        self.b = 0.0
        self.lux = 0.0
        self.calibration_lux = 0
        self.calibration_red = 0
        self.calibration_green = 0
        self.calibration_blue = 0
        self.calib_done = False
        self.last_measurment = [0]*3
        
        #convert integration time and gain if form that it has to be
        self.convert_integration()
        
        #set integration time and value
        self.tcs = Adafruit_TCS34725.TCS34725(
            integration_time=self.integration_time,gain=self.integration_gain)

        #Publisher
        self.msg = StreetLightController ()
        self.pub=rospy.Publisher("StreetLightController",StreetLightController,queue_size=1)

        
        #paths
        self.dirname = '/data/config/calibrations/street-light-controller/'
        self.filename = self.dirname  + self.veh_name + ".yaml"

        # delete old calibration
        if os.path.isdir(self.dirname):
            shutil.rmtree (self.dirname)
        
        #set light on and wait untill the lights are on for all WT
        self.msg.r_control_input=1.0
        self.msg.g_control_input=1.0
        self.msg.b_control_input=1.0
        self.pub.publish(self.msg)
        rospy.sleep(5)
        
        #calibration
        self.calibrator()
        
        #wait until calibration is done.
        while self.calib_done == False:
            rospy.sleep(0.5)
        
        #set calibration parameter
        self.set_param()
        
        #wait until all calibration are done and start the LED control in a clean way. 
        rospy.sleep(10)
        self.msg.r_control_input=0
        self.msg.g_control_input=0
        self.msg.b_control_input=0
        self.pub.publish(self.msg)

        
    def calibrator(self):
        r = []
        b = []
        g = []
        lux = []
        
        for j in range (self.num_calib):
            self.get_lux()
            lux.append(self.lux)
            r.append(self.r)
            g.append(self.g)
            b.append(self.b)
        
        #set value of calibration as reference
        self.calibration_lux = np.max(lux)
        self.calibration_red = np.max(r)
        self.calibration_green = np.max(g)
        self.calibration_blue = np.max(b)
        
        self.calib_done = True


    def get_lux(self):
        # Read R, G, B, C color data from the sensor.
        try:
            self.r,self.g, self.b, self.c = self.tcs.get_raw_data()
        except:
            pass
            
        if self.r >= 200:
            self.r = self.last_measurment[0]
        if self.g >= 200:
            self.g = self.last_measurment[1]
        if self.b >= 200: 
            self.b = self.last_measurment[2]
            
        self.last_measurment[0]=self.r
        self.last_measurment[1]=self.g
        self.last_measurment[2]=self.b

        # Calculate lux
        self.lux = Adafruit_TCS34725.calculate_lux(self.r, self.g, self.b)
        rospy.loginfo("r=%d, g=%d, b=%d, lux=%d" %(self.r, self.g, self.b, self.lux))

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

    def set_param(self):
        data = {
            "calibration_time": time.strftime("%Y-%m-%d-%H-%M-%S"),
            "lux": int(self.calibration_lux),
            "r": int(self.calibration_red),
            "g": int(self.calibration_green),
            "b": int(self.calibration_blue)
        }
        os.makedirs(self.dirname)
        with open(self.filename, 'w') as file:
            file.write(yaml.dump(data, default_flow_style=False))
            
        self.log("[%s] Saved calibration: calibration lux = %d, calibration red = %d, calibration green = %d, calibration blue = %d" % (self.node_name, self.calibration_lux,self.calibration_red, self.calibration_green, self.calibration_blue))


if __name__ == '__main__':
    # create the node
    node = Calibration(node_name='dt_calibration')
    
