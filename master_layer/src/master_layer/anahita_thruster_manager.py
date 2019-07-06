#!/usr/bin/env python

import numpy
import rospy
from geometry_msgs.msg import Wrench
from anahita_msgs.msg import Thrust

class AnahitaThrusterManager(object):

    def __init__(self, *arg, **kwargs):

        self.n_thrusters = 0
        self.configuration_matrix = None
        if rospy.has_param('/anahita_thruster_allocator/tam'):
            tam = rospy.get_param('/anahita_thruster_allocator/tam')
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

        self._max_pwm = 1100
        self._min_pwm = 1900

    def compute_pwm(self, thrust):
        return numpy.interp(thrust, self._input, self._output)

    def input_callback(self, msg):

        if not self.ready:
            return

        force = numpy.array((msg.force.x, msg.force.y, msg.force.z))
        torque = numpy.array((msg.torque.x, msg.torque.y, msg.torque.z))

        gen_forces = numpy.hstack((force, torque)).transpose()

        self.thrust = self.compute_thruster_forces(gen_forces)

	print(str(self.thrust))
        self.command_thrusters()

    def compute_thruster_forces(self, gen_forces):
        """Compute desired thruster forces using the inverse configuration
        matrix.
        """
        # Calculate individual thrust forces
        thrust = self.inverse_configuration_matrix.dot(gen_forces)
	rospy.loginfo("Thrust computed per thruster: " + str(thrust))
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
    thrust_input = [-88.29, -87.30900000000001, -85.347, -82.7964, -80.73630000000001, -78.28380000000001, -74.45790000000001, -71.41680000000001, -68.8662, -66.1194, -62.8821, -60.920100000000005, -58.5657, -54.64170000000001, -51.1101, -48.85380000000001, -47.088, -43.0659, -40.71150000000001, -38.259, -35.5122, -32.2749, -28.8414, -26.879400000000004, -24.230700000000002, -22.1706, -19.62, -17.0694, -15.009300000000001, -13.341600000000001, -11.2815, -9.4176, -7.651800000000001, -5.9841, -4.1202, -2.4525, -1.1772, -0.5886, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2753, 2.4525, 4.0221, 5.6898, 7.553700000000001, 9.025200000000002, 11.2815, 13.734, 15.499800000000002, 18.2466, 20.4048, 23.838300000000004, 26.977500000000003, 30.1167, 33.0597, 36.297000000000004, 39.043800000000005, 42.379200000000004, 45.3222, 47.9709, 52.18920000000001, 54.5436, 58.27140000000001, 61.11630000000001, 65.33460000000001, 69.06240000000001, 72.4959, 74.556, 75.537, 81.71730000000001, 85.2489, 86.62230000000001, 90.252, 94.66650000000001, 98.78670000000001, 103.8879, 105.94800000000001, 110.1663]
    pwm_output = [-400.0, -390.0, -380.0, -370.0, -360.0, -350.0, -340.0, -330.0, -320.0, -310.0, -300.0, -290.0, -280.0, -270.0, -260.0, -250.0, -240.0, -229.99999999999997, -220.00000000000003, -210.0, -200.0, -190.0, -180.0, -170.0, -160.0, -150.0, -140.0, -130.0, -120.0, -110.00000000000001, -100.0, -90.0, -80.0, -70.0, -60.0, -50.0, -40.0, -30.0, 0, 0, 0.0, 0, 0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.00000000000001, 120.0, 130.0, 140.0, 150.0, 160.0, 170.0, 180.0, 190.0, 200.0, 210.0, 220.00000000000003, 229.99999999999997, 240.0, 250.0, 260.0, 270.0, 280.0, 290.0, 300.0, 310.0, 320.0, 330.0, 340.0, 350.0, 360.0, 370.0, 380.0, 390.0, 400.0]
        
    anahita_thruster_manager = AnahitaThrusterManager(input=thrust_input, output=pwm_output)
    print 'Anahita Thruster Manager started'
    rospy.spin()


