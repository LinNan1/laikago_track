#!/usr/bin/python
import rospy
import message_filters
import cv2
import cv2.aruco as aruco
import sys
import thread
import dlib
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from pyzbar import pyzbar
from cv_bridge import CvBridge, CvBridgeError
from laikago_tracker.srv import tracking_target,tracking_targetResponse
from laikago_msgs.msg import HighCmd, HighState
from simple_pid import PID
# laikago_tracker/tracking_target

class Tracker:
    def __init__(self):
        self.track_res_pub = rospy.Publisher('/laikago_traker/img_res',Image,queue_size=10)
        self.high_cmd_pub  = rospy.Publisher('/laikago_real/high_cmd',HighCmd,queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = message_filters.Subscriber('/camera/color/image_raw',Image)
        self.depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw',Image)
        self.state_sub = message_filters.Subscriber('/laikago_real/high_state',HighState)
        self.time_syn = message_filters.ApproximateTimeSynchronizer([self.image_sub,self.depth_sub,self.state_sub], 10, 0.1, allow_headerless=True)
        self.time_syn.registerCallback(self.sub_callback)
        self.play_buffer = []
        self.mouse_down = False
        self.points_left_top_selected = []
        self.points_right_bottom_selected = []
        self.selected = False
        self.rect  = []
        self.track_screenshot = None
        # contol
        self.SendHighROS = HighCmd()
        self.RecvHighROS = HighState()
        self.P = 0.0005
        self.I = 0
        self.D = 0.00002
        self.pid_h = PID(self.P, self.I, self.D, setpoint=320)
        self.pid_h.output_limits = (-0.1, 0.1)
        self.cur_X = rospy.Publisher('/laikago_traker/cur_X',Int32,queue_size=10)
        self.tar_X = rospy.Publisher('/laikago_traker/tar_X',Int32,queue_size=10)
        self.pid_v = PID(self.P, self.I, self.D, setpoint=240)
        self.pid_v.output_limits = (-0.1, 0.1)
        self.cur_Y = rospy.Publisher('/laikago_traker/cur_Y',Int32,queue_size=10)
        self.tar_Y = rospy.Publisher('/laikago_traker/tar_Y',Int32,queue_size=10)

        self.safe_distance = 1 # m
        self.pid_dist = PID(0.8, 0.0, 0.0015, setpoint=self.safe_distance)
        self.pid_dist.output_limits = (-1.0,1.0)
    def algorithm_init(self):
        # correlation
        # self.object_tracker = dlib.correlation_tracker()
        # self.object_tracker.start_track(self.track_screenshot,dlib.rectangle(*self.rect[-1]))
 
        # meanshift
        xmin,ymin,xmax,ymax = (self.points_left_top_selected[-1][0],
                               self.points_left_top_selected[-1][1],
                               self.points_right_bottom_selected[-1][0],
                               self.points_right_bottom_selected[-1][1])
        self.track_wnd = (xmin,ymin,xmax-xmin,ymax-ymin)
        roi = self.track_screenshot[ymin:ymax,xmin:xmax] # region of interest
        hsv_roi = cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_roi, (0,60,32), (180,255,255))
        self.roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0,180])
        cv2.normalize(self.roi_hist, self.roi_hist,0,255,cv2.NORM_MINMAX)
        self.term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 50, 1)

        

    def algorithm_run(self,image):
        obj_center = (-1,-1)
        # correlation
        # self.object_tracker.update(image)
        # target_rect = self.object_tracker.get_position()
        # pt1 = (int(target_rect.left()), int(target_rect.top()))
        # pt2 = (int(target_rect.right()), int(target_rect.bottom()))
        # cv2.rectangle(image, pt1, pt2, (255, 255, 255), 3)

        # aruco
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
        if len(corners) > 0:
            ids = ids.flatten()
            for markerCorner,markerID in zip(corners,ids):
                if markerID == 11:
                    markerCorner = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = markerCorner
                    
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))

                    cv2.line(image, topLeft, topRight, (255, 255, 255), 2)
                    cv2.line(image, topRight, bottomRight, (255, 255, 255), 2)
                    cv2.line(image, bottomRight, bottomLeft, (255, 255, 255), 2)
                    cv2.line(image, bottomLeft, topLeft, (255, 255, 255), 2)
                    cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                    cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

                    obj_center = (cX,cY)
        # meanshift
        # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # dst = cv2.calcBackProject([hsv],[0],self.roi_hist,[0,180],1)
        # retval, self.track_wnd = cv2.meanShift(dst,self.track_wnd,self.term_crit)
        # x,y,w,h = self.track_wnd
        # cv2.rectangle(image, (x,y), (x+w,y+h), (255, 255, 255), 3)


        image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        image = cv2.resize(image, (320,240), interpolation = cv2.INTER_AREA)
        return image, obj_center

    def sendHighCmd(self,mode=0,forwardSpeed=0.0,sideSpeed=0.0,rotateSpeed=0.0,roll=0.0,pitch=0.0,yaw=0.0):
        self.SendHighROS.mode = mode
        self.SendHighROS.forwardSpeed = forwardSpeed
        self.SendHighROS.sideSpeed = sideSpeed
        self.SendHighROS.rotateSpeed = rotateSpeed

        self.SendHighROS.roll  = roll
        self.SendHighROS.pitch = pitch
        self.SendHighROS.yaw = yaw

        self.high_cmd_pub.publish(self.SendHighROS)

    def sub_callback(self,image,depth,state):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth, "passthrough")

        except CvBridgeError as e:
            print(e)

        if len(self.play_buffer) > 5:
            self.play_buffer.pop(0)
            self.play_buffer.append(cv_image)
        else:
            self.play_buffer.append(cv_image)  # select img


        depth_array = np.array(depth_image, dtype=np.float32)
        
        cv_image, target_center = self.algorithm_run(cv_image)
        
        
        safe_distance = self.safe_distance

        self.SendHighROS.mode = 0
        self.SendHighROS.forwardSpeed = 0.0
        self.SendHighROS.sideSpeed = 0.0
        self.SendHighROS.rotateSpeed = 0.0
        
        if target_center != (-1,-1):
            distance = depth_array[target_center[1]][target_center[0]]/1000
            if distance <= safe_distance:
                self.SendHighROS.mode = 0
            elif distance > safe_distance:
                self.SendHighROS.mode = 2
                self.SendHighROS.roll  = 0
                self.SendHighROS.pitch = 0
                self.SendHighROS.yaw = 0
                
                self.pid_h.reset()
                self.pid_v.reset()

                self.SendHighROS.forwardSpeed = self.pid_dist(-distance)
                if self.SendHighROS.forwardSpeed > 1.0:
                    self.SendHighROS.forwardSpeed = 1.0
                if self.SendHighROS.forwardSpeed < -1.0:
                    SendHighROS.forwardSpeed = -1.0

                rospy.loginfo("speed: %f distance: %f",self.SendHighROS.forwardSpeed,distance)

            if state.mode == 0:
                
                self.SendHighROS.yaw -= self.pid_h(target_center[0])

                self.cur_X.publish(target_center[0])
                self.tar_X.publish(320)

                if self.SendHighROS.yaw < -1.0: self.SendHighROS.yaw = -1.0
                elif self.SendHighROS.yaw > 1.0: self.SendHighROS.yaw = 1.0

                self.SendHighROS.pitch -= self.pid_v(target_center[1])

                self.cur_Y.publish(target_center[1])
                self.tar_Y.publish(240)

                if self.SendHighROS.pitch < -1.0: self.SendHighROS.pitch = -1.0
                elif self.SendHighROS.pitch > 1.0: self.SendHighROS.pitch = 1.0
                # rospy.loginfo("pitch: %f  distance: %f" % (self.SendHighROS.pitch,distance))

        self.high_cmd_pub.publish(self.SendHighROS)

        if self.selected:
            cv_image, target_center = self.algorithm_run(cv_image)

        # TO DO
        

        try:
            self.track_res_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "passthrough"))
        except CvBridgeError as e:
             print(e)
        

    def select_target_thread(self):
        # Step 1: screen shot
        window_1 = 'Press "p" to puase'
        cv2.namedWindow(window_1,cv2.WINDOW_AUTOSIZE)
        while True:
            if len(self.play_buffer) > 0:
                color_image = self.play_buffer.pop(0)
                cv2.imshow(window_1,color_image)
            key = cv2.waitKey(1)
            
            if key & 0xFF == ord('p') or key == 27:
                break
        # Step 2: select object
        cv2.setWindowTitle(window_1,'Press "s" to select')
        cv2.imshow(window_1,color_image)
        mouse_active = [True]
        def on_mouse(event,x,y,flags,param):
            if mouse_active[-1]:
                if event == cv2.EVENT_LBUTTONDOWN:
                    self.mouse_down = True
                    self.points_left_top_selected.append((x,y))
                    print('left-top: ({},{})'.format(x,y))
                elif event == cv2.EVENT_LBUTTONUP and self.mouse_down == True:
                    self.mouse_down = False
                    self.points_right_bottom_selected.append((x,y))
                    print('right-bottom: ({},{})'.format(x,y))
                elif event == cv2.EVENT_MOUSEMOVE and self.mouse_down == True:
                    im_draw = color_image.copy()
                    cv2.rectangle(im_draw, self.points_left_top_selected[-1], (x, y), (255,255,255), 3)
                    cv2.imshow(window_1, im_draw)
        cv2.setMouseCallback(window_1, on_mouse)
        while True:
            if len(self.points_left_top_selected) == len(self.points_right_bottom_selected) and len(self.points_left_top_selected):
                # xmin,ymin,xmax,ymax
                self.rect.append((self.points_left_top_selected[-1][0],
                                  self.points_left_top_selected[-1][1],
                                  self.points_right_bottom_selected[-1][0],
                                  self.points_right_bottom_selected[-1][1]))
            key = cv2.waitKey(1)
            if key & 0xFF == ord('s') or key == 27:
                mouse_active.append(False)
                break
        cv2.destroyWindow(window_1)
        self.track_screenshot = color_image
        # Step 3: track setup
        self.algorithm_init()
        self.selected = True
    

def main(args):
    rospy.init_node('tracker', anonymous=True)
    tracker = Tracker()
    # thread.start_new_thread(tracker.select_target_thread,())
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
   
if __name__ == '__main__':
    main(sys.argv)