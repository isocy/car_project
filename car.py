from threading import Thread
import time

import RPi.GPIO as GPIO


def control_leftMotor():
    pass

def read_dist(TRIG, ECHO, dist_list, dist_idx):
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)
    GPIO.output(TRIG, GPIO.LOW)
    
    dist_hist = []
    
    while True:
        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(TRIG, GPIO.LOW)
        
        while GPIO.input(ECHO) == GPIO.LOW:
            start_time = time.time()
        while GPIO.input(ECHO) == GPIO.HIGH:
            stop_time = time.time()
        cur_dist = ((stop_time - start_time) * 340 * 100) / 2
        
        if len(dist_hist) < 5:
            dist_hist.append(cur_dist)
        else:
            dist_hist.pop(0)
            dist_hist.append(cur_dist)
        
            sum = 0
            min, max = 400, 0
            for dist in dist_hist:
                sum += dist
                if dist < min:
                    min = dist
                if dist > max:
                    max = dist
            
            dist_list[dist_idx] = (sum - min - max) / 3
            print(f"idx {dist_idx} / dist {dist_list[dist_idx]}")


if __name__ == '__main__':
    LEFTMOTOR_PIN = 12
    RIGHTMOTOR_PIN = 13
    LEFT_TRIG = 2
    LEFT_ECHO = 3
    FRONT_TRIG = 4
    FRONT_ECHO = 14
    RIGHT_TRIG = 15
    RIGHT_ECHO = 18
    
    LEFT_DIST_IDX = 0
    FRONT_DIST_IDX = 1
    RIGHT_DIST_IDX = 2
    
    PWM_FREQ = 100
    
    MAX_DISTANCE = 400
    dist = [MAX_DISTANCE for i in range(3)]
    
    GPIO.setmode(GPIO.BCM)
    
    
    leftSonic_thread = Thread(target=read_dist, args=(LEFT_TRIG, LEFT_ECHO, dist, LEFT_DIST_IDX))
    frontSonic_thread = Thread(target=read_dist, args=(FRONT_TRIG, FRONT_ECHO, dist, FRONT_DIST_IDX))
    rightSonic_thread = Thread(target=read_dist, args=(RIGHT_TRIG, RIGHT_ECHO, dist, RIGHT_DIST_IDX))
#    leftMotor_thread = Thread(target=control_leftMotor, args=(dist,))

    leftSonic_thread.start()
    frontSonic_thread.start()
    rightSonic_thread.start()
    
    leftSonic_thread.join()
    frontSonic_thread.join()
    rightSonic_thread.join()
    
    
#    pwm_left = GPIO.PWM(LEFTMOTOR_PIN, PWM_FREQ)
#    pwm_right = GPIO.PWM(RIGHTMOTOR_PIN, PWM_FREQ)