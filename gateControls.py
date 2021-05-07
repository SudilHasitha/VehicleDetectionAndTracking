import RPi.GPIO as GPIO
import time 

class gateControls:
        def __init__(self):
            self.motorPins = (7,25,5,6 )   # define pins connected to four phase ABCD of stepper motor)
            self.CCWStep = (0x01,0x02,0x04,0x08) # define power supply order for rotating anticlockwise 
            self.destroyCWStep = (0x08,0x04,0x02,0x01)  # define power supply order for rotating clockwise
            self.CWStep = (0x08,0x04,0x02,0x01)  # define power supply order for rotating clockwise
            GPIO.setmode(GPIO.BCM)       # use PHYSICAL GPIO Numbering
            for pin in self.motorPins:
                GPIO.setup(pin,GPIO.OUT)
            
        def setup(self):    
            GPIO.setmode(GPIO.BCM)       # use PHYSICAL GPIO Numbering
            for pin in self.motorPins:
                GPIO.setup(pin,GPIO.OUT)
                
        # as for four phase stepping motor, four steps is a cycle. the function is used to drive the stepping motor clockwise or anticlockwise to take four steps    
        def moveOnePeriod(self,direction,ms):    
            for j in range(0,4,1):      # cycle for power supply order
                for i in range(0,4,1):  # assign to each pin
                    if (direction == 1):# power supply order clockwise
                        GPIO.output(self.motorPins[i],((self.CCWStep[j] == 1<<i) and GPIO.HIGH or GPIO.LOW))
                    else :              # power supply order anticlockwise
                        GPIO.output(self.motorPins[i],((self.CWStep[j] == 1<<i) and GPIO.HIGH or GPIO.LOW))
                if(ms<3):       # the delay can not be less than 3ms, otherwise it will exceed speed limit of the motor
                    ms = 3
                time.sleep(ms*0.001)    
                
        # continuous rotation function, the parameter steps specifies the rotation cycles, every four steps is a cycle
        def moveSteps(self,direction, ms, steps):
            for i in range(steps):
                self.moveOnePeriod(direction, ms)
                
        # function used to stop motor
        def motorStop(self):
            for i in range(0,4,1):
                GPIO.output(self.motorPins[i],GPIO.LOW)
                    
        def loop(self):
            while True:
                self.moveSteps(1,3,512)  # rotating 360 deg clockwise, a total of 2048 steps in a circle, 512 cycles
                time.sleep(0.5)
                self.moveSteps(0,3,512)  # rotating 360 deg anticlockwise
                time.sleep(0.5)

        def destroy(self):
            GPIO.cleanup()             # Release resource


        
