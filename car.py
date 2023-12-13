from threading import Thread, Lock
import time

import RPi.GPIO as GPIO

forward_limit = 30
side_limit = 30
lock = Lock()

GPIO.setwarnings(False)
LEFT_MOTOR = 12
RIGHT_MOTOR = 13
LEFT_TRIG = 17
LEFT_ECHO = 27
FRONT_TRIG = 6
FRONT_ECHO = 26
RIGHT_TRIG = 5
RIGHT_ECHO = 23
in1 = 22
in2 =24
in3 =16 
in4 =10
LEFT_MOTOR = 18
RIGHT_MOTOR = 8
LEFT_DIST_IDX = 0
FRONT_DIST_IDX = 1
RIGHT_DIST_IDX = 2
dist = [0 for i in range(3)]
def read_dist(TRIG, ECHO, dist_list, dist_idx):
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)
    GPIO.output(TRIG, GPIO.LOW)
    
    dist_hist = []
    
    while True:
        time.sleep(0.01)
        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(TRIG, GPIO.LOW)
        
        temp_start = GPIO.input(ECHO)
        start_time = time.time()
        start_pre_time = start_time
        while temp_start == GPIO.LOW:
            temp_start = GPIO.input(ECHO)
            start_time = time.time()
            if start_time-start_pre_time > 0.1:
                temp_start = GPIO.HIGH
        temp_end = GPIO.input(ECHO)
        stop_time = time.time()
        stop_pre_time = stop_time
        while temp_end == GPIO.HIGH:
            temp_end = GPIO.input(ECHO)
            stop_time = time.time()
            if stop_time - stop_pre_time > 0.1:
                temp_end = GPIO.LOW
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
            
            avg = (sum - min - max) / 3
            dist_list[dist_idx] = avg
            
        

def forward(left_pwm,right_pwm):
    left_pwm.start(50)
    right_pwm.start(50)
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)
def turn_left(left_pwm,right_pwm):
    left_pwm.start(50)
    right_pwm.start(50)
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)
    
def turn_right(left_pwm,right_pwm):
    left_pwm.start(50)
    right_pwm.start(50)
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
    
def backward(left_pwm,right_pwm):
    left_pwm.start(50)
    right_pwm.start(50)
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
    
if __name__ == '__main__':
    
    
    
    
    PWM_FREQ=1000
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(in1,GPIO.OUT)
    GPIO.setup(in2,GPIO.OUT)
    GPIO.setup(in3,GPIO.OUT)
    GPIO.setup(in4,GPIO.OUT)
    GPIO.setup(LEFT_MOTOR,GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR,GPIO.OUT)
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)
    
    leftSonic_thread = Thread(target=read_dist, args=(LEFT_TRIG, LEFT_ECHO, dist, LEFT_DIST_IDX))
    frontSonic_thread = Thread(target=read_dist, args=(FRONT_TRIG, FRONT_ECHO, dist, FRONT_DIST_IDX))
    rightSonic_thread = Thread(target=read_dist, args=(RIGHT_TRIG, RIGHT_ECHO, dist, RIGHT_DIST_IDX))

    leftSonic_thread.start()
    frontSonic_thread.start()
    rightSonic_thread.start()


    GPIO.setup(LEFT_MOTOR, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR, GPIO.OUT)
    left_pwm = GPIO.PWM(LEFT_MOTOR, PWM_FREQ)
    right_pwm = GPIO.PWM(RIGHT_MOTOR, PWM_FREQ)
    

    
    left_pwm.start(50)
    right_pwm.start(50)
    
    try:   
        while True:
            time.sleep(0.03)
            if dist[FRONT_DIST_IDX]>forward_limit and dist[RIGHT_DIST_IDX]>side_limit and dist[LEFT_DIST_IDX]>side_limit:
                forward(left_pwm,right_pwm)
                
            elif dist[FRONT_DIST_IDX]>forward_limit and dist[RIGHT_DIST_IDX]>side_limit and dist[LEFT_DIST_IDX]<=side_limit:
                turn_right(left_pwm,right_pwm)
                
            elif dist[FRONT_DIST_IDX]>forward_limit and dist[RIGHT_DIST_IDX]<=side_limit and dist[LEFT_DIST_IDX]>side_limit:
                turn_left(left_pwm,right_pwm)   
            elif dist[FRONT_DIST_IDX]<=forward_limit and dist[RIGHT_DIST_IDX]>side_limit and dist[LEFT_DIST_IDX]<=side_limit:
                turn_right(left_pwm,right_pwm)
            elif dist[FRONT_DIST_IDX]<=forward_limit and dist[RIGHT_DIST_IDX]<=side_limit and dist[LEFT_DIST_IDX]>side_limit:
                turn_left(left_pwm,right_pwm)
            elif dist[FRONT_DIST_IDX]<=forward_limit and dist[RIGHT_DIST_IDX]<=side_limit and dist[LEFT_DIST_IDX]<=side_limit:
                turn_right(left_pwm,right_pwm)

                    
    except KeyboardInterrupt:
        GPIO.cleanup()
        exit(0)
                

