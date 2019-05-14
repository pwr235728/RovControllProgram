from RovPID import *
import time

pitch_pid_params_in = PidParams(Kp=1.0, Ki=0.0, Kd=0.0, Limit=positionPID_limit)
pitch_pid_params_out = PidParams(Kp=1.0, Ki=0.0, Kd=0.0, Limit=speedPID_limit)

pitch_pid = RovPID(pitch_pid_params_out, pitch_pid_params_in, 0.01)


p = 0.95
q = 1-p


speed = 0.0
position = 0

last_time = time.time()
current_time = time.time()
pitch_pid.SetPoint = 90.0
while True:
    dt = current_time - last_time
    last_time = current_time
    current_time = time.time()

    speed = speed*p + pitch_pid.update(position, speed) * q
    position = position + speed*0.01

    print(speed, "; ", position)
    time.sleep(0.01)
