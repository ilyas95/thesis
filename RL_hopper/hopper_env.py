from openai_ros import robot_gazebo_env
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry


class HopperEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all Robot environments.
    """

    def __init__(self):
        """Initializes a new Robot environment.
        """
        #HAA hip abduction/adduction - HFE hip flexion extension - KFE knee flexion/extension
        self.controllers_list = ['haa_joint_position_controller','hfe_joint_position_controller','kfe_joint_position_controller']
        self.robot_name_space = "monoped"
        reset_controls_bool = True
        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(HopperEnv, self).__init__(controllers_list=self.controllers_list,
                                                robot_name_space=self.robot_name_space,
                                                reset_controls=reset_controls_bool)
        #Unpause simulation
        self.gazebo.unpauseSim()
        self.controllers_object.reset_controllers()
        self._check_all_sensors_ready()
        # We use it to get the joints positions and calculate the reward associated to it
        rospy.Subscriber("/monoped/joint_states", JointState, self._joints_callback)
        # Odom we only use it for the height detection and planar position because in real robots this data is not trivial.
        rospy.Subscriber("/odom", Odometry, self._odom_callback)
        # We use the IMU for orientation and linearacceleration detection
        rospy.Subscriber("/monoped/imu/data", Imu, self._imu_callback)
        # We use it to get the contact force, to know if its in the air or stumping too hard.
        rospy.Subscriber("/lowerleg_contactsensor_state", ContactsState, self._contact_callback)
        #Publishers
        self._haa_joint_pub = rospy.Publisher('/monoped/haa_joint_position_controller/command', Float64, queue_size=1)
        self._hfe_joint_pub = rospy.Publisher('/monoped/hfe_joint_position_controller/command', Float64, queue_size=1)
        self._kfe_joint_pub = rospy.Publisher('/monoped/kfe_joint_position_controller/command', Float64, queue_size=1)
        self.publishers_array = [self._haa_joint_pub, self._hfe_joint_pub, self._kfe_joint_pub]
        #Pause simulation
        self._check_publishers_connection()
        self.gazebo.pauseSim()

    # Methods needed by the RobotGazeboEnv
    # ----------------------------

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        self._check_all_sensors_ready()
        self._check_publishers_connection()
        return True
    
    # ----------------------------  
    def _check_all_sensors_ready(self):
        self._check_odom_ready()
        self._check_imu_ready()
        self._check_contacts_ready()
        self._check_joint_states_ready()
        rospy.logdebug("ALL SENSORS READY")
    
    def _check_joint_states_ready(self):
        self.joints = None
        while self.joints is None and not rospy.is_shutdown():
            try:
                self.joints = rospy.wait_for_message("/monoped/joint_states", JointState, timeout=1.0)
                rospy.logdebug("Current /monoped/joint_states READY=>" + str(self.joints))
            except:
                rospy.logerr("Current /monoped/joint_states not ready yet, retrying for getting joint_states")
        return self.joints
    
    def _check_odom_ready(self):
        self.odom = None
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message("/odom", Odometry, timeout=1.0)
                rospy.logdebug("Current /odom READY=>" + str(self.odom))
            except:
                rospy.logerr("Current /odom not ready yet, retrying for getting odom")
        return self.odom

    def _check_imu_ready(self):
        self.imu = None 
        while self.imu is None and not rospy.is_shutdown():
            try:
                self.imu = rospy.wait_for_message("/monoped/imu/data",Imu,timeout=1.0)    
                rospy.logdebug("Current /monoped/imu/data READY=>" + str(self.imu))
            except:
                rospy.logerr("Current /monoped/imu/data not ready yet, retrying for getting imu")
        return self.imu

    def _check_contacts_ready(self):
        self.contact = None
        while self.contact is None and not rospy.is_shutdown():
            try:
                self.contact = rospy.wait_for_message("/lowerleg_contactsensor_state",ContactsState,timeout=1.0)
                rospy.logdebug("Current /lowerleg_contactsensor_state READY=>" + str(self.contact))
            except:
                rospy.logerr("Current /lowerleg_contactsensor_state not ready yet, retrying for getting contact")
        return self.contact

    def _joints_callback(self, data):
        self.joints = data
    
    def _odom_callback(self, data):
        self.odom = data

    def _imu_callback(self, data):
        self.imu = data

    def _contact_callback(self, data):
        for state in data.states:
            self.contact_force = state.total_wrench.force

    def get_joints(self):
        return self.joints

    def get_odom(self):
        return self.odom 

    def _check_publishers_connection(self):
        rate = rospy.Rate(10)
        while (self._haa_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _haa_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_haa_joint_pub Publisher Connected")
        while (self._hfe_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _hfe_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_hfe_joint_pub Publisher Connected")
        while (self._kfe_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _kfe_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_kfe_joint_pub Publisher Connected")
        rospy.logdebug("All Publishers READY") 

    # Methods that the TrainingEnvironment will need to define
    # ----------------------------
    
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()
    
    
    def _init_env_variables(self):
        """Inits variables needed to be initialized each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()
        
    # Methods that the TrainingEnvironment will need.
    # ----------------------------

    def move_joints(self, joints):
        self.publishers_array[0].publish(joints[0])
        self.publishers_array[1].publish(joints[1])
        self.publishers_array[2].publish(joints[2])