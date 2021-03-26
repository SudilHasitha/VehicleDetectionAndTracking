#Libraries
import RPi.GPIO as GPIO
import time

def distance():   
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
    
        TRIG = 23
        ECHO = 24
        maxTime = 0.04

        while True:
            GPIO.setup(TRIG,GPIO.OUT)
            GPIO.setup(ECHO,GPIO.IN)
    
            GPIO.output(TRIG,False)
    
            time.sleep(1)
    
            GPIO.output(TRIG,True)
    
            time.sleep(0.00001)
    
            GPIO.output(TRIG,False)
    
            pulse_start = time.time()
            timeout = pulse_start + maxTime
            while GPIO.input(ECHO) == 0 and pulse_start < timeout:
                pulse_start = time.time()
    
            pulse_end = time.time()
            timeout = pulse_end + maxTime
            while GPIO.input(ECHO) == 1 and pulse_end < timeout:
                pulse_end = time.time()
    
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150
            distance = round(distance, 2)
    
            print(distance,"cm")
    except:
        GPIO.cleanup()

if __name__ == '__main__':
    distance()
            