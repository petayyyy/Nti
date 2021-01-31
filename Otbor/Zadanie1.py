import rospy
from clover import srv
from std_srvs.srv import Trigger
from pyzbar import pyzbar
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
c = 0
x_last = 0
y_last = 0
go = True
rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

bridge = CvBridge()

#rospy.init_node('barcode_test')


navigate(x=0, y=0, z=1, speed=1, frame_id='body', auto_arm=True)
rospy.sleep(4)

navigate(x=2, y=0, z=0.5, speed=1, frame_id='aruco_map')
rospy.sleep(4)

# Image subscriber callback function
def image_callback(data):
    global x_last
    global y_last
    global c
    global go
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    barcodes = pyzbar.decode(cv_image)
    for barcode in barcodes:
        b_data = barcode.data.encode("utf-8")
        x = float(b_data[0])
        y = float(b_data[-1])
        if (x != x_last and y != y_last):
            c += 1
            x_last = x
            y_last = y
            print ("QR code number: {}    x = {}, y = {}".format(c, x, y))
            navigate(x=x, y=y, z=0.5, speed=20, frame_id='aruco_map')
            rospy.sleep(5)
            if c == 3:
                land()
                go = False

def ros_time():
    global go
    while go:
        rospy.rostime.wallsleep(0.01)

def start():
    image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)
    ros_time()

start()
