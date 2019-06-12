
from cv_bridge import CvBridge
import cv2
import sys
import roslib, rospy
roslib.load_manifest('rosbag')
import rosbag

TOPIC = '/anahita/front_camera/image_raw'

def CreateVideoBag(videopath, bagname):
    '''Creates a bag file with a video file'''
    bag = rosbag.Bag(bagname, 'w')
    cap = cv2.VideoCapture(videopath)
    cb = CvBridge()
    try:
        prop_fps = cap.get(cv2.cv.CV_CAP_PROP_FPS)
        if prop_fps != prop_fps or prop_fps <= 1e-2:
            print "Warning: can't get FPS. Assuming 15."
            prop_fps = 15
    except: 
        print "Warning: can't get FPS. Assuming 15."
        prop_fps = 15
    ret = True
    frame_id = 0
    while(ret):
        ret, frame = cap.read()
        if not ret:
            break
        stamp = rospy.rostime.Time.from_sec(float(frame_id) / prop_fps)
        frame_id += 1
        image = cb.cv2_to_imgmsg(frame, encoding='bgr8')
        image.header.stamp = stamp
        image.header.frame_id = "camera"
        bag.write(TOPIC, image, stamp)
    cap.release()
    bag.close()

if __name__ == "__main__":
    if len( sys.argv ) == 3:
        CreateVideoBag(*sys.argv[1:])
    else:
        print( "Usage: video2bag videofilename bagfilename")
