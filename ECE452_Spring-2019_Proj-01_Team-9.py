#!/usr/bin/python
# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO
import time
from AlphaBot import AlphaBot

cntl = 8;
cntr = 7;

# Initial value for Encoders
EncR = 0.0;
EncL = 0.0;

CS = 5
Clock = 25
Address = 24
DataOut = 23

   
class TRSensor(object):
    def __init__(self,numSensors = 5):
        self.numSensors = numSensors
        self.calibratedMin = [0] * self.numSensors
        self.calibratedMax = [1023] * self.numSensors
        self.last_value = 0

    
    def AnalogRead(self):
        value = [0,0,0,0,0,0]
        #Read Channel0~channel4 AD value
        for j in range(0,6):
            GPIO.output(CS, GPIO.LOW)
            for i in range(0,4):
                #sent 4-bit Address
                if(((j) >> (3 - i)) & 0x01):
                    GPIO.output(Address,GPIO.HIGH)
                else:
                    GPIO.output(Address,GPIO.LOW)
                #read MSB 4-bit data
                value[j] <<= 1
                if(GPIO.input(DataOut)):
                    value[j] |= 0x01
                GPIO.output(Clock,GPIO.HIGH)
                GPIO.output(Clock,GPIO.LOW)
            for i in range(0,6):
                #read LSB 8-bit data
                value[j] <<= 1
                if(GPIO.input(DataOut)):
                    value[j] |= 0x01
                GPIO.output(Clock,GPIO.HIGH)
                GPIO.output(Clock,GPIO.LOW)
            #no mean ,just delay
            for i in range(0,6):
                GPIO.output(Clock,GPIO.HIGH)
                GPIO.output(Clock,GPIO.LOW)
           #time.sleep(0.0001)
            GPIO.output(CS,GPIO.HIGH)
        return value[1:]

    
    def calibrate(self):
        max_sensor_values = [0]*self.numSensors
        min_sensor_values = [0]*self.numSensors
        for j in range(0,10):

            sensor_values = self.AnalogRead();

            for i in range(0,self.numSensors):

                # set the max we found THIS time
                if((j == 0) or max_sensor_values[i] < sensor_values[i]):
                    max_sensor_values[i] = sensor_values[i]

                # set the min we found THIS time
                if((j == 0) or min_sensor_values[i] > sensor_values[i]):
                    min_sensor_values[i] = sensor_values[i]

        # record the min and max calibration values
        for i in range(0,self.numSensors):
            if(min_sensor_values[i] > self.calibratedMin[i]):
                self.calibratedMin[i] = min_sensor_values[i]
            if(max_sensor_values[i] < self.calibratedMax[i]):
                self.calibratedMax[i] = max_sensor_values[i]

    def readCalibrated(self):
        value = 0
        #read the needed values
        sensor_values = self.AnalogRead();

        for i in range (0,self.numSensors):

            denominator = self.calibratedMax[i] - self.calibratedMin[i]

            if(denominator != 0):
                value = (sensor_values[i] - self.calibratedMin[i])* 1000 / denominator

            if(value < 0):
                value = 0
            elif(value > 1000):
                value = 1000

            sensor_values[i] = value

        print("readCalibrated",sensor_values)
        return sensor_values

   
    def readLine(self, white_line = 0):

        sensor_values = self.readCalibrated()
        avg = 0
        sum = 0
        on_line = 0
        for i in range(0,self.numSensors):
            value = sensor_values[i]
            if(white_line):
                value = 1000-value
            # keep track of whether we see the line at all
            if(value > 200):
                on_line = 1

            # only average in values that are above a noise threshold
            if(value > 50):
                avg += value * (i * 1000);  # this is for the weighted total,
                sum += value;                  #this is for the denominator

        if(on_line != 1):
            # If it last read to the left of center, return 0.
            if(self.last_value < (self.numSensors - 1)*1000/2):
                #print("left")
                return 0;

            # If it last read to the right of center, return the max.
            else:
                #print("right")
                return (self.numSensors - 1)*1000

        self.last_value = avg/sum

        return self.last_value

def updateEncoderL(channel):
    global EncL;
    EncL += 1;#print 'valEncL = %d' %EncL

def updateEncoderR(channel):
    global EncR;
    EncR += 1;#print 'valEncR = %d' %EncR
    
   
# Initialising the Pins of raspberry pi to sensors.

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(Clock,GPIO.OUT)
GPIO.setup(Address,GPIO.OUT)
GPIO.setup(CS,GPIO.OUT)
GPIO.setup(DataOut,GPIO.IN,GPIO.PUD_UP)
GPIO.setup(cntr, GPIO.IN);
GPIO.setup(cntl, GPIO.IN);
GPIO.add_event_detect(cntr, GPIO.BOTH, updateEncoderR)
GPIO.add_event_detect(cntl, GPIO.BOTH, updateEncoderL)


# the main function which excutes the line tracking and stopping distance
if __name__ == '__main__':

    from AlphaBot import AlphaBot

    #while True:
        # the car starts after its being pushed
        #if EncR > 1 or EncL > 1:
            #break
            
    maximum = 25;
    integral = 0;
    last_proportional = 0

    TR = TRSensor()
    Ab = AlphaBot()
    Ab.stop()
    print("Line follow Example")
    time.sleep(0.5)
    for i in range(0,400):
        TR.calibrate()
        print (i)

    #print(TR.calibratedMin)
    #print(TR.calibratedMax)
    time.sleep(0.5)
    Ab.backward()

    while True:

        position = TR.readLine()
        #print(position)

        # The "proportional" term should be 0 when we are on the line.
        proportional = position - 2000

        # Compute the derivative (change) and integral (sum) of the position.
        derivative = proportional - last_proportional
        integral += proportional

        # Remember the last position.
        last_proportional = proportional

        '''
        //Compute the difference between the two motor power settings, m1 - m2. 
        //If this is a positive number the robot will turn to the right. 
        //If it is a negative number, the robot will turn to the left.
        //The magnitude of the number determines the sharpness of the turn. 
        '''
        power_difference = proportional/25 + derivative/100 #+ integral/1000;

        if (power_difference > maximum):
            power_difference = maximum
        if (power_difference < - maximum):
            power_difference = - maximum
        #print(position,power_difference)
        if (power_difference < 0):
            Ab.setPWMB(maximum + power_difference)
            Ab.setPWMA(maximum);
        else:
            Ab.setPWMB(maximum);
            Ab.setPWMA(maximum - power_difference)

        #Encoder program to stop at 50 inch.
        R0 = EncR;
        nt = 50;          #50 inch
        diameter = 8;     # wheel's diameter
        distanceR = (EncR/40) * 8;
        distanceL = (EncL/40) * 8;
        
        # Prints the distance travelled by the car and displays it to the PC
        if (distanceR > distanceL):
            print ('distance = %d' %distanceR);   
        else:
            print ('distance = %d' %distanceL);
            
        while distanceR > nt or distanceL > nt:
            Ab.stop();
