from rospy import ServiceProxy
from ros_ws281x.srv import SetLeds
from ros_ws281x.msg import LEDState, LEDStateArray
from std_msgs.msg import ColorRGBA

# Количество светодиодов в ленте
NUM_LEDS = 60

set_leds = ServiceProxy("/led/set_leds", SetLeds, persistent=True)


def led(color): # 0=yellow, 1=green, 2=blue, 3=red, -1=norhing
    led_msg = LEDStateArray()
    led_msg.leds = []
    
    red, green, blue = 0, 0, 0
    if color == 0:
        red, green, blue = 255, 255, 0
    elif color == 1:
        red, green, blue = 0, 255, 0
    elif color == 2:
        red, green, blue = 0, 0, 255
    elif color == 3:
        red, green, blue = 255, 0, 0
    elif color == -1:
        red, green, blue = 0, 0, 0
        
    for i in range(NUM_LEDS): # Количество светодиодов в ленте
        led = LEDState(i, ColorRGBA(red, green, blue, 0))
        led_msg.leds.append(led) # Записываем состояние светодиода в сообщение

    set_leds(led_msg)
