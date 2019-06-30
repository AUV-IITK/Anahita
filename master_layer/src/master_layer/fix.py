#!/usr/bin/env python

import numpy
import pandas
import os
import h5py

def fix_left (features):
    m_x = features[2]
    m_r_h = features[10]
    r_y = features[5]
    r_l = features[8]
    m_r_v = features[13]

    l_x = m_x - m_r_h
    l_y = r_y
    l_l = 0.9*r_l
    l_m_h = m_r_h
    l_r_h = 2*m_r_h
    l_m_v = m_r_v
    l_r_v = 0

    features[0] = int(l_x)
    features[1] = int(l_y)
    features[6] = int(l_l)
    features[9] = int(l_m_h)
    features[12] = int(l_m_v)
    features[11] = int(l_r_h)
    features[14] = int(l_r_v)

    # print ("left: {}, {}".format(features, m_r_v))

def fix_middle(features):
    l_x = features[0]
    l_y = features[1]
    r_x = features[4]
    r_y = features[5]
    l_l = features[6]
    r_l = features[8]
    l_r_h = features[11]

    m_x = (l_x + r_x)/2
    m_y = (l_y + r_y)/2 - 0.15*(l_l + r_l)
    m_l = 0.2*(l_l + r_l)
    l_m_h = l_r_h/2
    m_r_h = l_r_h/2
    l_m_v = 0.15*(l_l + r_l)
    m_r_v = 0.15*(l_l + r_l)

    features[2] = int(m_x)
    features[3] = int(m_y)
    features[7] = int(m_l)
    features[9] = int(l_m_h)
    features[10] = int(m_r_h)
    features[12] = int(l_m_v)
    features[13] = int(m_r_v)

    # print ("middle: {}".format(features))

def fix_right(features):
    m_x = features[2]
    l_m_h = features[9]
    l_y = features[1]
    l_l = features[6]
    l_m_v = features[12]

    r_x = m_x + l_m_h
    r_y = l_y
    r_l = 0.9*l_l
    m_r_h = l_m_h
    m_r_v = l_m_v
    l_r_h = 2*l_m_h
    l_r_v = 0

    features[4] = int(r_x)
    features[5] = int(r_y)
    features[8] = int(r_l)
    features[10] = int(m_r_h)
    features[13] = int(m_r_v)
    features[11] = int(l_r_h)
    features[14] = int(l_r_v)

    # print ("right: {}".format(features))

def make_string (features, dist):
    str_ = ''
    for i in range(len(features)):
        str_ += str(features[i]) + ' '
    str_ = str_ + str(dist) + '\n'
    return str_

if __name__ == '__main__':
    num_attr = 15 # number of features fed into the network

    # load dataset
    dataframe = pandas.read_csv("../../data/gate_yaw_test.csv", delim_whitespace=True, header=None)
    dataset = dataframe.values
    print ('data loaded')

    # split into input (X) and output (Y) variables
    X = dataset[:, 0:num_attr]
    Y = dataset[:,num_attr]

    f = open('../../data/gate_yaw_test1.csv', 'w+')

    for i in range(len(X)):
        features = X[i]

        if features[0] == -1:
            fix_left (features)
        elif features[2] == -1:
            fix_middle (features)
        elif features[4] == -1:
            fix_right (features)

        str_ = make_string (features, Y[i])
        f.write(str_)
        f.flush()

    f.close()