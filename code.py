import cv2
import math
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover.srv import SetLEDEffect

rospy.init_node('computer_vision_sample')
bridge = CvBridge()
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

cascadePath = "haarcascade_frontalface_default.xml"
face_cascade_db = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
color = (0, 0, 255)
xl, yl, sl, sn, xn, yn, flagx, flagy, flagz = 320, 240, 0, 0, 0, 0, 0, 0, 0

def takeoff(z=1.5):
    navigate(z=z, speed=0.5, frame_id='body', auto_arm=True)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            break
        rospy.sleep(0.2)

def image_callback(data):
    global xl, yl, sl, xn, yn, sn,flagx,flagy,flagz
    img=bridge.imgmsg_to_cv2(data, 'bgr8')
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_gray = cv2.resize(img_gray, (0, 0), fx=0.5, fy=0.5)
    faces = face_cascade_db.detectMultiScale(img_gray, 1.1, 19)
    x, y, w, h = 0, 0, 0, 0

    flagx = 0
    flagy = 0
    flagz = 0
    error = 0

    if faces != ():
        set_effect(r=0, g=255, b=0)
        error = 0
        for (x, y, w, h) in faces:

            #smeshenie litsa
            cv2.rectangle(img, (x*2, y*2), ((x + w)*2, (y + h)*2), (0, 255, 0), 2)
            xn, yn, sn = round(x + (w / 2))*2, round(y + (h / 2))*2, h*2
            cv2.putText(img, "{}, {}, {}".format(xn, yn, sn), (50, 260), cv2.FONT_HERSHEY_SIMPLEX, 2, color, 2)
            if sl == 0:
                sl = sn
            # x:
            if xn - xl > 250:
                flagx = 1
            elif xn - xl > 10:
                cv2.putText(img, "left", (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
                flagx = -2
            elif xn - xl < -10:
                cv2.putText(img, "right", (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
                flagx = 2
            # y:
            if yn - yl > 250:
                flagy = 1
            elif yn - yl > 10:
                cv2.putText(img, "down", (0, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
                flagy = -2
            elif yn - yl < -10:
                cv2.putText(img, "up", (0, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
                flagy = 2
            # z:
            if sn - sl > 5:
                cv2.putText(img, "backward", (0, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
                flagz = -2
            elif sn - sl < -5:
                cv2.putText(img, "forward", (0, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
                flagz = 2
            else:
                flagz = 1
            break

        #smeshenie v nujnuu storonu
        speed = 0.5
        frame_id = 'body'
        auto_arm = False
        
        if (flagx==1) and (flagy==1) and (flagz == 1):
            pass
        elif flagx==2:
            navigate(x=0.001,speed=speed,frame_id=frame_id,auto_arm=auto_arm)
        elif flagy==2:
            navigate(y=0.001,speed=speed,frame_id=frame_id,auto_arm=auto_arm)
        elif flagz==2:
            navigate(z=0.001,speed=speed,frame_id=frame_id,auto_arm=auto_arm)
        elif flagx==-2:
            navigate(x=-0.001,speed=speed,frame_id=frame_id,auto_arm=auto_arm)
        elif flagy==-2:
            navigate(y=-0.001,speed=speed,frame_id=frame_id,auto_arm=auto_arm)
        elif flagz==-2:
            navigate(z=-0.001,speed=speed,frame_id=frame_id,auto_arm=auto_arm)
        rospy.sleep(0.01)

    else:
        set_effect(r=255, g=0, b=0)
        cv2.putText(img, "NAN", (257, 260), cv2.FONT_HERSHEY_SIMPLEX, 2, color, 2)
        error+=1
        rospy.sleep(1)
        if error >=10:
            land()
        sl = 0

    print(faces)
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

image_sub = rospy.Subscriber('front_camera/image_raw_throttled', Image, image_callback)
image_pub = rospy.Publisher('~debug', Image)
takeoff()
rospy.spin()







