from RovPID import *
import time

speedPID_limit = 100  # maksymalna moc - ograniczenie wyjścia PID od prędkości
positionPID_limit = 90  # maksymalna prędkość - ograniczenie wyjścia PID od pozycji

pitch_pid_params_out = PidParams(Kp=5.0, Ki=0.0, Kd=0.0, Limit=positionPID_limit)
pitch_pid_params_in = PidParams(Kp=5.0, Ki=0.0, Kd=0.0, Limit=speedPID_limit)

pitch_pid = RovPID(pitch_pid_params_out, pitch_pid_params_in, sample_time=0.01)


# super simple simulation

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

    print(speed, "; ", position, "; ", pitch_pid.positionPID.output, "; ", pitch_pid.speedPID.output)
    time.sleep(0.01)
