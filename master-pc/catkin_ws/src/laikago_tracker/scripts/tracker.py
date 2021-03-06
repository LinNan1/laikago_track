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
from std_msgs.msg import Int32,Float32
from cv_bridge import CvBridge, CvBridgeError
from laikago_tracker.srv import tracking_target,tracking_targetResponse
from laikago_msgs.msg import HighCmd, HighState
from simple_pid import PID
# laikago_tracker/tracking_targe

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
        self.selected =  False
        self.initBB = None
        # contol
        self.SendHighROS = HighCmd()
        self.RecvHighROS = HighState()
        self.P = 0.0005
        self.I = 0
        self.D = 0.00002
        self.pid_yaw = PID(self.P, self.I, self.D, setpoint=320)
        self.pid_yaw.output_limits = (-0.1, 0.1)
        self.pid_pitch = PID(self.P, self.I, self.D, setpoint=240)
        self.pid_pitch.output_limits = (-0.1, 0.1)

        self.pid_sideSpeed = PID(0.000002, 0.0, 0.0000002, setpoint=320)
        self.pid_sideSpeed.output_limits = (-1.0, 1.0)
        self.pid_rotateSpeed = PID(0.000103833984375, 0.0, 0.0, setpoint=320)
        self.pid_rotateSpeed.output_limits = (-1.0, 1.0)

        self.safe_distance = 1 # m
        self.pid_forwardSpeed = PID(1.0, 0.0, 0.0, setpoint=-self.safe_distance)
        self.pid_forwardSpeed.output_limits = (-1.0,1.0)

        self.control_flag = False
        self.control_duration = 0
        self.control_start_undetected = rospy.Time.now()

        self.cur_X = rospy.Publisher('/laikago_traker/cur_X',Int32,queue_size=10)
        self.tar_X = rospy.Publisher('/laikago_traker/tar_X',Int32,queue_size=10)
        self.cur_Y = rospy.Publisher('/laikago_traker/cur_Y',Int32,queue_size=10)
        self.tar_Y = rospy.Publisher('/laikago_traker/tar_Y',Int32,queue_size=10)
        self.cur_distance = rospy.Publisher('/laikago_traker/cur_distance',Float32,queue_size=10)
        self.tar_distance = rospy.Publisher('/laikago_traker/tar_distance',Float32,queue_size=10)
        self.cmd_yaw = rospy.Publisher('/laikago_traker/cmd_yaw',Float32,queue_size=10)
        self.cmd_pitch = rospy.Publisher('/laikago_traker/cmd_pitch',Float32,queue_size=10)
        self.cmd_rotateSpeed = rospy.Publisher('/laikago_traker/cmd_rotateSpeed',Float32,queue_size=10)
        
    def select_target_thread(self):
        window_1 = 'Press "s" to select'
        cv2.namedWindow(window_1,cv2.WINDOW_AUTOSIZE)
        while True:
            if len(self.play_buffer) > 0:
                color_image = self.play_buffer.pop(0)
                cv2.imshow(window_1,color_image)
            key = cv2.waitKey(1)
            if key & 0xFF == ord('s') or key == 27:
                cv2.setWindowTitle(window_1,'Select a ROI and then press SPACE or ENTER button!')
                self.initBB = cv2.selectROI(window_1,color_image,fromCenter=False,showCrosshair=True)
                break
        cv2.destroyWindow(window_1)
        self.algorithm_init(color_image)
        self.selected = True

    def algorithm_init(self,image):
        x,y,w,h = self.initBB
        # correlation
        # self.object_tracker = dlib.correlation_tracker()
        # self.object_tracker.start_track(image,dlib.rectangle(x,y,x+w,y+h))
 
        # meanshift
        # xmin,ymin,xmax,ymax = x,y,x+w,y+h
        # self.track_wnd = (x,y,w,h)
        # roi = image[ymin:ymax,xmin:xmax] # region of interest
        # hsv_roi = cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
        # mask = cv2.inRange(hsv_roi, (0,60,32), (180,255,255))
        # self.roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0,180])
        # cv2.normalize(self.roi_hist, self.roi_hist,0,255,cv2.NORM_MINMAX)
        # self.term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 50, 1)

        # opencv tracker
        # self.object_tracker = cv2.TrackerKCF_create()
        # self.object_tracker = cv2.TrackerBoosting_create()
        # self.object_tracker = cv2.TrackerMIL_create()
        # self.object_tracker = cv2.TrackerTLD_create()
        # self.object_tracker = cv2.TrackerMedianFlow_create()
        self.object_tracker = cv2.TrackerMOSSE_create()
        self.object_tracker.init(image,self.initBB)

    def algorithm_run(self,image):
        obj_center = (-1,-1)

        # aruco
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        # parameters =  aruco.DetectorParameters_create()
        # corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
        # if len(corners) > 0:
        #     ids = ids.flatten()
        #     for markerCorner,markerID in zip(corners,ids):
        #         if markerID == 11:
        #             markerCorner = markerCorner.reshape((4, 2))
        #             (topLeft, topRight, bottomRight, bottomLeft) = markerCorner
        #             topRight = (int(topRight[0]), int(topRight[1]))
        #             bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        #             bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        #             topLeft = (int(topLeft[0]), int(topLeft[1]))
        #             cv2.line(image, topLeft, topRight, (255, 255, 255), 2)
        #             cv2.line(image, topRight, bottomRight, (255, 255, 255), 2)
        #             cv2.line(image, bottomRight, bottomLeft, (255, 255, 255), 2)
        #             cv2.line(image, bottomLeft, topLeft, (255, 255, 255), 2)
        #             cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
        #             cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        #             cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        #             cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
        #             obj_center = (cX,cY)

        # correlation
        # self.object_tracker.update(image)
        # target_rect = self.object_tracker.get_position()
        # pt1 = (int(target_rect.left()), int(target_rect.top()))
        # pt2 = (int(target_rect.right()), int(target_rect.bottom()))
        # cX, cY = (pt1[0]+pt2[0])/2, (pt1[1]+pt2[1])/2
        # cX = 640-1 if cX >= 640 else cX
        # cY = 480-1 if cY >= 480 else cY
        # obj_center = (cX,cY)
        # cv2.rectangle(image, pt1, pt2, (255, 255, 255), 3)

        # meanshift
        # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # dst = cv2.calcBackProject([hsv],[0],self.roi_hist,[0,180],1)
        # retval, self.track_wnd = cv2.meanShift(dst,self.track_wnd,self.term_crit)
        # x,y,w,h = self.track_wnd
        # cX, cY = x+w/2, y+h/2
        # cX = 640-1 if cX >= 640 else cX
        # cY = 480-1 if cY >= 480 else cY
        # obj_center = (cX,cY)
        # cv2.rectangle(image, (x,y), (x+w,y+h), (255, 255, 255), 3)

        # opencv tracker
        (success, box) = self.object_tracker.update(image)
        if success:
            (x,y,w,h) = [int(v) for v in box]
            cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,255),3)
            cX, cY = x+w/2, y+h/2
            cX = 640-1 if cX >= 640 else cX
            cY = 480-1 if cY >= 480 else cY
            obj_center = (cX,cY)

        # image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        # image = cv2.resize(image, (320,240), interpolation = cv2.INTER_AREA)
        return image, obj_center

    def sub_callback(self,image,depth,state):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth, "passthrough")
            depth_array = np.array(depth_image, dtype=np.float32)
            target_center = (-1,-1)
        except CvBridgeError as e:
            print(e)

        if len(self.play_buffer) > 5:
            self.play_buffer.pop(0)
            self.play_buffer.append(cv_image)
        else:
            self.play_buffer.append(cv_image)  # select img

        safe_distance = self.safe_distance
        if self.selected:  
            cv_image, target_center = self.algorithm_run(cv_image)
        
        if target_center != (-1,-1):
            self.control_start_undetected = rospy.Time.now()
            distance = depth_array[target_center[1]][target_center[0]]/1000 + 0.000000001
            tmp = 0.1
            if safe_distance - 0.2 <= distance <= safe_distance:
                # if (target_center[0] < (320 - tmp*320) or target_center[0] > (320 + tmp*320)) and self.control_flag and (self.control_duration < 100):
                if (target_center[0] < (320 - 0.01*320) or target_center[0] > (320 + 0.01*320)) and self.control_flag:
                    self.SendHighROS.mode = 2
                    self.SendHighROS.forwardSpeed = 0.0
                    self.SendHighROS.sideSpeed += self.pid_sideSpeed(target_center[0]) / distance
                    self.SendHighROS.rotateSpeed += self.pid_rotateSpeed(target_center[0])
                    self.control_duration += 1
                else:
                    self.control_flag = False
                    self.control_duration = 0
                    self.SendHighROS.mode = 1
                    self.SendHighROS.forwardSpeed = 0.0
                    self.SendHighROS.sideSpeed = 0.0
                    self.SendHighROS.rotateSpeed = 0.0
                    self.pid_forwardSpeed.reset()
                    self.pid_sideSpeed.reset()
                    self.pid_rotateSpeed.reset()
                    if state.forwardSpeed == 0.0:
                        self.SendHighROS.forwardSpeed = 0.0
                        self.SendHighROS.sideSpeed = 0.0
                        self.SendHighROS.rotateSpeed = 0.0
                        self.pid_forwardSpeed.reset()
                        self.pid_sideSpeed.reset()
                        self.pid_rotateSpeed.reset()
                        

                        self.SendHighROS.yaw -= self.pid_yaw(target_center[0])
                        if self.SendHighROS.yaw < -1.0: self.SendHighROS.yaw = -1.0
                        elif self.SendHighROS.yaw > 1.0: self.SendHighROS.yaw = 1.0
                        self.SendHighROS.pitch -= self.pid_pitch(target_center[1])
                        if self.SendHighROS.pitch < -1.0: self.SendHighROS.pitch = -1.0
                        elif self.SendHighROS.pitch > 1.0: self.SendHighROS.pitch = 1.0

                        if self.SendHighROS.yaw < -0.6 or self.SendHighROS.yaw > 0.6:
                            self.control_flag = True
                            self.SendHighROS.mode = 2
                            self.SendHighROS.forwardSpeed = 0.0
                            self.SendHighROS.sideSpeed += self.pid_sideSpeed(target_center[0]) / distance
                            self.SendHighROS.rotateSpeed += self.pid_rotateSpeed(target_center[0])

                    else:
                        self.SendHighROS.mode = 1
                        self.SendHighROS.roll  = 0
                        #self.SendHighROS.pitch = 0
                        self.SendHighROS.yaw = 0
                        self.pid_pitch.reset()
                        self.pid_yaw.reset()
                    
            else:
                self.control_flag = False

                self.SendHighROS.yaw = 0

                self.SendHighROS.mode = 2
                self.SendHighROS.forwardSpeed = self.pid_forwardSpeed(-distance)
                self.SendHighROS.sideSpeed += self.pid_sideSpeed(target_center[0]) /distance
                self.SendHighROS.rotateSpeed += self.pid_rotateSpeed(target_center[0])
            
            self.cur_X.publish(target_center[0])
            self.tar_X.publish(320)
            self.cur_Y.publish(target_center[1])
            self.tar_Y.publish(240)
            self.cur_distance.publish(distance)
            self.tar_distance.publish(self.safe_distance)

        elif rospy.Time.now().to_nsec() - self.control_start_undetected.to_nsec() > 500000000:
            self.SendHighROS.mode = 1
            self.SendHighROS.forwardSpeed = 0
            self.SendHighROS.rotateSpeed = 0
            self.SendHighROS.sideSpeed = 0

        self.high_cmd_pub.publish(self.SendHighROS)

        self.cmd_yaw.publish(self.SendHighROS.yaw)
        self.cmd_pitch.publish(self.SendHighROS.pitch)
        self.cmd_rotateSpeed.publish(self.SendHighROS.rotateSpeed)

        try:
            self.track_res_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "passthrough"))
        except CvBridgeError as e:
             print(e)
        


    

def main(args):
    rospy.init_node('tracker', anonymous=True)
    tracker = Tracker()
    thread.start_new_thread(tracker.select_target_thread,())
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
   
if __name__ == '__main__':
    main(sys.argv)