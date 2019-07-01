#!/usr/bin/env python

import numpy
import rospy
import os
import h5py
import time
from std_msgs.msg import Int32MultiArray, String
from keras.models import model_from_json
from master_layer.srv import GetMaxDepth, GetMaxDepthResponse
from master_layer.srv import TargetNormal, TargetNormalResponse

z_avg = 0
z_coord = numpy.zeros(10)
z_count = 0
current_task = ""

yaw_avg = 0
yaw_coord = numpy.zeros(10)
yaw_count = 0

features = []
depth_model = 'gate_depth'
yaw_model = 'gate_yaw'

features_init =False

MODEL_DIR = '/home/ironman/anahita_ws/src/Anahita/master_layer/models'

def load_model (model_name):
    json_filename = os.path.join(MODEL_DIR, model_name + '.json')
    json_file = open(json_filename, 'r')
    loaded_model_json = json_file.read()
    json_file.close()
    loaded_model = model_from_json(loaded_model_json)
    weights_filename = os.path.join(MODEL_DIR, model_name + '.h5')
    loaded_model.load_weights(weights_filename)
    return loaded_model

models = dict()

models['gate_depth'] = load_model ('gate_depth_model')
# models['buoy_depth'] = load_model ('buoy_depth_model')
# models['buoy_yaw'] = load_model ('buoy_yaw_model')
# models['marker'] = load_model ('marker_depth_model')
models['gate_yaw'] = load_model ('gate_yaw_model')

def in_range (a, b, thres):
    if abs(a - b) < thres:
        return True
    return False

def avg (array):
    sum_ = numpy.sum(array)
    size = len(array)
    return sum_/size
  
def depth_avg (depth):
    global z_coord
    global z_avg
    global z_count

    z = depth
    if (z_count < 10):
        z_coord[z_count] = z
        z_avg = avg (z_coord)
        z_count += 1
    else:
        z_avg = avg (z_coord)
        if (in_range(z, z_avg, 10)):
            z_coord = numpy.roll(z_coord, -1)
            z_coord[9] = z

def yaw_avg_ (yaw):
    global yaw_coord
    global yaw_avg
    global yaw_count

    if (yaw_count < 10):
        yaw_coord[yaw_count] = yaw
        yaw_avg = avg (yaw_coord)
        yaw_count += 1
    else:
        yaw_avg = avg (yaw_coord)
        if (in_range(yaw, yaw_avg, 2)):
            yaw_coord = numpy.roll(yaw_coord, -1)
            yaw_coord[9] = yaw

def predict_ (model, features):
    X = numpy.array([features])
    value = models[model].predict(X)[0][0]
    return value

def featuresCB (msg):
    global features
    global features_init
    features_init = True
    features = []
    size = len(msg.data)

    for i in range(size):
        features.append(msg.data[i])

def currentTaskCB (msg):
    global current_task
    global depth_model
    global yaw_model

    current_task = msg.data

    if current_task == 'gate':
        depth_model = 'gate_depth'
        yaw_model = 'gate_yaw'
    if current_task == 'buoy':
        depth_model = 'buoy_depth'
    if current_task == 'marker':
        depth_model = 'marker_depth'

def getDepthCB (request):
    return GetMaxDepthResponse(z_avg)

def getNormal (request):
    return TargetNormalResponse(yaw_avg/57.11)

if __name__ == '__main__':
    
    rospy.init_node('depth_node')
    features_sub = rospy.Subscriber ('/anahita/features', Int32MultiArray, featuresCB)
    current_task_sub = rospy.Subscriber ('/anahita/current_task', String, currentTaskCB)

    get_max_depth = rospy.Service('/anahita/get_max_depth', GetMaxDepth, getDepthCB)
    target_normal = rospy.Service('/anahita/target_normal', TargetNormal, getNormal)

    while not rospy.is_shutdown():
        if features_init:
            try:
                depth = predict_(depth_model, features)
                depth_avg (depth)
                yaw = predict_(yaw_model, features)
                yaw_avg_ (yaw)
                print (yaw_avg)
            except:
                print ('something bad happened')
        time.sleep(0.1)