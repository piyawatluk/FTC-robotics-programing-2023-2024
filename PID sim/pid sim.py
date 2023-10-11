import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import logging

logging.basicConfig(filename="log.txt", filemode="w")
logger = logging.getLogger()
logger.setLevel(logging.INFO)

cd = str(input("sim code"))
sa = float(input("start angle: "))
sp = float(input("end angle: "))
t = int(input("time limit: "))
kp = float(input("kp index: "))
ki = float(input("ki index: "))
tau = float(input("scan_index: "))
re = float(input("refresh_time: "))
delay_current = float(input("angle error vs time: "))
fua = float(input("fuckup_angle: "))
mp = int(input("maximum accept angle: "))
mm = int(input("minimum accept angle: "))
cof = float(input("turn index: "))

logger.info("sim start")
logger.info("sim code %s", cd)
logger.info("value of sa: %s", sa)
logger.info("value of sp: %s", sp)
logger.info("value of t: %s", t)
logger.info("value of kp: %s", kp)
logger.info("value of ki: %s", ki)
logger.info("value of tau: %s", tau)
logger.info("value of re: %s", re)
logger.info("value of delay: %s", delay_current)
logger.info("value of fua: %s", fua)
logger.info("value of mp: %s", mp)
logger.info("value of mm: %s", mm)

time_points = [0]
current_value = sa
values = [current_value]
intregal_value = [0]
por_value = [0]
i = 0

target_approached = False

while i < t:
    error_index = sp - current_value
    pr = kp * error_index
    intri = (ki / tau) * error_index
    pl = pr + intri
    current_value += pl * cof + fua
    values.append(current_value)
    i += re + delay_current
    if not target_approached and mm <= current_value < mp:
        print('target approach', 'time', i, "angle approach", current_value)
        logger.info("target approach time %s angle approach %s",i,current_value)
        target_approached = True
    if current_value > mp:
        print("angle overshoot above maximum accept angle for", current_value , "at time", i)
        logger.critical("Angle overshoot above maximum accepted angle for %s at time %s", current_value, i)
    if (current_value - mp) > 5:
        print("recommend adjusting k index")
        logger.critical("recommend adjusting k index")
    else:
        print("k index is okay")
        logger.info("k index is okay")
    time_points.append(i)
    por_value.append(pr)
    intregal_value.append(intri)

new_time_points = np.linspace(0, t, num=1000)

control_output_interp = interp1d(time_points, values, kind='cubic')
proportional_interp = interp1d(time_points, por_value, kind='cubic')
integral_interp = interp1d(time_points, intregal_value, kind='cubic')

smoothed_values = control_output_interp(new_time_points)
smoothed_por_value = proportional_interp(new_time_points)
smoothed_intregal_value = integral_interp(new_time_points)

plt.figure(figsize=(8, 6))
plt.plot(new_time_points, smoothed_values, label='Control Output (Smoothed)')
plt.plot(new_time_points, smoothed_por_value, label='Proportional Term (Smoothed)')
plt.plot(new_time_points, smoothed_intregal_value, label='Integral Term (Smoothed)')
plt.plot(time_points, values, label='Control Output ')
plt.plot(time_points, por_value, label='Proportional Term ')
plt.plot(time_points, intregal_value, label='Integral Term ')
plt.xlabel("Time")
plt.ylabel("Control Output")
plt.title("Control Output and PID Components vs. Time" + cd)
plt.legend()
plt.grid(True)
plt.show()
