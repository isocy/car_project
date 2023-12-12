from threading import Thread, Lock
import time

import RPi.GPIO as GPIO


lock = Lock()

GPIO.setwarnings(False)


def read_dist(TRIG, ECHO, dist_list, dist_idx):
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)
    GPIO.output(TRIG, GPIO.LOW)
    
    dist_hist = []
    
    while True:
        time.sleep(0.3)
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
            
            avg = (sum - min - max) / 3
            dist_list[dist_idx] = avg
        
            lock.acquire()
            print(f"idx {dist_idx} / dist {avg}")
            lock.release()


if __name__ == '__main__':
    LEFT_MOTOR_PWM = 12
    RIGHT_MOTOR_PWM = 13
    LEFT_MOTOR_INPUT0 = 10
    LEFT_MOTOR_INPUT1 = 9
    RIGHT_MOTOR_INPUT0 = 11
    RIGHT_MOTOR_INPUT1 = 8
    LEFT_TRIG = 17
    LEFT_ECHO = 27
    FRONT_TRIG = 4
    FRONT_ECHO = 14
    RIGHT_TRIG = 22
    RIGHT_ECHO = 23
    
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

    leftSonic_thread.start()
    frontSonic_thread.start()
    rightSonic_thread.start()


    GPIO.setup(LEFT_MOTOR_PWM, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_PWM, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_INPUT0, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_INPUT1, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_INPUT0, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_INPUT1, GPIO.OUT)
    left_pwm = GPIO.PWM(LEFT_MOTOR_PWM, PWM_FREQ)
    right_pwm = GPIO.PWM(RIGHT_MOTOR_PWM, PWM_FREQ)
    
    GPIO.output(LEFT_MOTOR_INPUT0, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_INPUT1, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_INPUT0, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_INPUT1, GPIO.LOW)
    left_pwm.start(30)
    right_pwm.start(30)
    
    while True:
        time.sleep(0.03)
        if dist[FRONT_DIST_IDX] > 5:
            GPIO.output(LEFT_MOTOR_INPUT0, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_INPUT1, GPIO.LOW)
            GPIO.output(RIGHT_MOTOR_INPUT0, GPIO.HIGH)
            GPIO.output(RIGHT_MOTOR_INPUT1, GPIO.LOW)
        elif dist[LEFT_DIST_IDX] > dist[RIGHT_DIST_IDX]:
            GPIO.output(LEFT_MOTOR_INPUT0, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_INPUT1, GPIO.HIGH)
            GPIO.output(RIGHT_MOTOR_INPUT0, GPIO.HIGH)
            GPIO.output(RIGHT_MOTOR_INPUT1, GPIO.LOW)
        else:
            GPIO.output(LEFT_MOTOR_INPUT0, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_INPUT1, GPIO.LOW)
            GPIO.output(RIGHT_MOTOR_INPUT0, GPIO.LOW)
            GPIO.output(RIGHT_MOTOR_INPUT1, GPIO.HIGH)