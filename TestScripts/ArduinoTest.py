from ProjectPath import PROJECT_PATH
from ClassFiles.Arduino import Arduino

AD = Arduino()
alpha = 1
pwm = 230
AD.ManualPWM_Value = [pwm * 2 * alpha,pwm * 2 * (1-alpha),pwm]
AD.ManualPWM()