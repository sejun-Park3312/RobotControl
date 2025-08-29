from ProjectPath import PROJECT_PATH
from ClassFiles.Arduino import Arduino

AD = Arduino()
alpha = 0
pwm = 130
AD.ManualPWM_Value = [pwm * (1-alpha),pwm * (1+alpha),pwm]
AD.ManualPWM()