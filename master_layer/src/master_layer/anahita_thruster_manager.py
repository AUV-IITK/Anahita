#!/usr/bin/env python

import numpy
import rospy
from geometry_msgs.msg import Wrench
from anahita_msgs.msg import Thrust

class AnahitaThrusterManager(object):

    def __init__(self, *arg, **kwargs):

        self.n_thrusters = 0
        self.configuration_matrix = None
        if rospy.has_param('~tam'):
            tam = rospy.get_param('~tam')
            self.configuration_matrix = numpy.array(tam)
            self.n_thrusters = self.configuration_matrix.shape[1]

            rospy.loginfo('Thruster allocation matrix provided!')
            rospy.loginfo('TAM=')
            rospy.loginfo(self.configuration_matrix)

            self.inverse_configuration_matrix = None
            if self.configuration_matrix is not None:
                self.inverse_configuration_matrix = numpy.linalg.pinv(
                    self.configuration_matrix)
                rospy.loginfo('Inverse TAM=')
                rospy.loginfo(self.inverse_configuration_matrix)

        self.input_sub = rospy.Subscriber('/anahita/thruster_manager/input',
                                          Wrench, self.input_callback)
        self.pwm_pub = rospy.Publisher('/pwm', Thrust, queue_size=1, latch=True)

        self.thrust = None
        self.thrust = numpy.zeros(self.n_thrusters)

        self.ready = True

        if 'input' not in kwargs or 'output' not in kwargs:
            rospy.ROSException('Thruster input/output sample points not given')

        self._input = kwargs['input']
        self._output = kwargs['output']

        self._max_pwm = 1900
        self._min_pwm = 1100


    def compute_pwm(self, thrust):
        return numpy.interp(thrust, self._input, self._output)

    def input_callback(self, msg):

        if not self.ready:
            return

        force = numpy.array((msg.force.x, msg.force.y, msg.force.z))
        torque = numpy.array((msg.torque.x, msg.torque.y, msg.torque.z))

        gen_forces = numpy.hstack((force, torque)).transpose()

        self.thrust = self.compute_thruster_forces(gen_forces)
        self.command_thrusters()

    def compute_thruster_forces(self, gen_forces):
        """Compute desired thruster forces using the inverse configuration
        matrix.
        """
        # Calculate individual thrust forces
        thrust = self.inverse_configuration_matrix.dot(gen_forces)
        return thrust

    def command_thrusters(self):
        """Publish the thruster input into their specific topic."""
        if self.thrust is None:
            return

        pwm = Thrust()

        pwm.forward_left = int(self.compute_pwm(self.thrust[1]))
        pwm.forward_right = int(self.compute_pwm(self.thrust[0]))
        pwm.sideward_back = int(self.compute_pwm(self.thrust[3]))
        pwm.sideward_front = int(self.compute_pwm(self.thrust[2]))
        pwm.upward_north_east = int(self.compute_pwm(self.thrust[6]))
        pwm.upward_north_west = int(self.compute_pwm(self.thrust[5]))
        pwm.upward_south_east = int(self.compute_pwm(self.thrust[7]))
        pwm.upward_south_west = int(self.compute_pwm(self.thrust[4]))

        self.pwm_pub.publish(pwm)

if __name__ == '__main__':

    rospy.init_node('anahita_thruster_manager')
    thrust_input = [-9, -8.9, -8.7, -8.44, -8.23, -7.98, -7.59, -7.28, 
                    -7.02, -6.74, -6.41, -6.21, -5.97, -5.57, -5.21, 
                    -4.98, -4.8, -4.39, -4.15, -3.9, -3.62, -3.29, -2.94,
                    -2.74, -2.47, -2.26, -2, -1.74, -1.53, -1.36, -1.15, 
                    -0.96, -0.78, -0.61, -0.42, -0.25, -0.12, -0.06, 0, 0,
                    0, 0, 0, 0.13, 0.25, 0.41, 0.58, 0.77, 0.92, 1.15, 1.4,
                    1.58, 1.86, 2.08, 2.43, 2.75, 3.07, 3.37, 3.7, 3.98, 
                    4.32, 4.62, 4.89, 5.32, 5.56, 5.94, 6.23, 6.66, 7.04, 
                    7.39, 7.6, 7.7, 8.33, 8.69, 8.83, 9.2, 9.65, 10.07, 
                    10.59, 10.8, 11.23]

    pwm_output = [1100, 1110, 1120, 1130, 1140, 1150, 1160, 1170, 1180,
                1190, 1200, 1210, 1220, 1230, 1240, 1250, 1260, 1270,
                1280, 1290, 1300, 1310, 1320, 1330, 1340, 1350, 1360,
                1370, 1380, 1390, 1400, 1410, 1420, 1430, 1440, 1450,
                1460, 1470, 1480, 1490, 1500, 1510, 1520, 1530, 1540,
                1550, 1560, 1570, 1580, 1590, 1600, 1610, 1620, 1630, 
                1640, 1650, 1660, 1670, 1680, 1690, 1700, 1710, 1720,
                1730, 1740, 1750, 1760, 1770, 1780, 1790, 1800, 1810,
                1820, 1830, 1840, 1850, 1860, 1870, 1880, 1890, 1900]

    anahita_thruster_manager = AnahitaThrusterManager(input=thrust_input, output=pwm_output)
    print 'Anahita Thruster Manager started'
    rospy.spin()

