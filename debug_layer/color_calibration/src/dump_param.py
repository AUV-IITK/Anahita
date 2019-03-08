
from color_calibration.srv import *
import rospy
import subprocess
import yaml

def dump_paramters(req):
    print "Dumping Request, filename: [%s]"%(req.filename)
    env = rospy.get_param('/env')

    r_min = rospy.get_param('/color_calibrate/r_min')
    r_max = rospy.get_param('/color_calibrate/r_max')
    g_min = rospy.get_param('/color_calibrate/g_min')
    g_max = rospy.get_param('/color_calibrate/g_max')
    b_min = rospy.get_param('/color_calibrate/b_min')
    b_max = rospy.get_param('/color_calibrate/b_max')

    opening_mat_point = rospy.get_param('/color_calibrate/opening_mat_point')
    opening_iter = rospy.get_param('/color_calibrate/opening_iter')
    closing_iter = rospy.get_param('/color_calibrate/closing_iter')
    closing_mat_point = rospy.get_param('/color_calibrate/closing_mat_point')

    bilateral_iter = rospy.get_param('/color_calibrate/bilateral_iter')

    params = {
        'vision': {
            req.filename: {
                "r_min": r_min,
                "r_max": r_max,
                "g_min": g_min,
                "g_max": g_max,
                "b_min": b_min,
                "b_max": b_max,
                "opening_mat_point": opening_mat_point,
                "opening_iter": opening_iter,
                "closing_iter": closing_iter,
                "closing_mat_point": closing_mat_point,
                "bilateral_iter": bilateral_iter
            }
        }
    }
    
    file_location = "../params/" + req.filename + "_" + env + ".yaml"
    stream = file(file_location, 'w')
    yaml.dump(params, stream,  default_flow_style=False)
    return DumpResponse(True)

def save_parameters():
    rospy.init_node('dump_node')
    s = rospy.Service('dump_parameters', Dump, dump_paramters)
    print "Ready to save parameters"
    rospy.spin()

if __name__ == "__main__":
    save_parameters()