import robots.ultrabot_realtime_pwm as ultrabot_realtime

print "\nv2pwm: \n"
print ultrabot_realtime.v2pwm(20, 20)
print ultrabot_realtime.v2pwm(-4.79936880622, 4.79936880622)
print ultrabot_realtime.v2pwm(-1.0, 1.0)
print ultrabot_realtime.v2pwm(-0.0, 0.0)

print "\npwm2v: \n"
print ultrabot_realtime.pwm2v(99, 77)
print ultrabot_realtime.pwm2v(47, 80)
print ultrabot_realtime.pwm2v(-47, -80)
