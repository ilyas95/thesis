from gym import spaces
import hopper_env
from gym.envs.registration import register
import rospy
from tf.transformations import euler_from_quaternion
import numpy


timestep_limit_per_episode = 1000
register(
        id='HopperStayUp-v0',
        entry_point='hopper_stay_up_env:HopperStayUpEnv',
        timestep_limit=timestep_limit_per_episode,
    )


class HopperStayUpEnv(hopper_env.HopperEnv):
    def __init__(self):
        self.get_params()
        number_actions = rospy.get_param('/monoped/n_actions')
        self.action_space = spaces.Discrete(number_actions)
        super(HopperStayUpEnv, self).__init__()

    def get_params(self):
        self.init_joint = rospy.get_param("/monoped/init_joint_pose")
        self.desired_pose = rospy.get_param("/monoped/desired_pose")
        self.min_height = rospy.get_param("/monoped/min_height")
        self.max_height = rospy.get_param("/monoped/max_height")
        self.desired_force = rospy.get_param("/monoped/desired_force")
        self.desired_yaw = rospy.get_param("/monoped/desired_yaw")
        self.max_incl = rospy.get_param("/monoped/max_incl")
        self.joint_increment_value = rospy.get_param("/monoped/joint_increment_value")
        # Weights
        self.weight1 = rospy.get_param("/monoped/weight1") #joint positions
        self.weight2 = rospy.get_param("/monoped/weight2") #joint efforts
        self.weight3 = rospy.get_param("/monoped/weight3") #contact force
        self.weight4 = rospy.get_param("/monoped/weight4") #orientation
        self.weight5 = rospy.get_param("/monoped/weight5") #distance from desired position
        #Rewards
        self.alive_reward = rospy.get_param("/monoped/alive_reward")
        self.done_reward = rospy.get_param("/monoped/done_reward")
        #Limits
        self.haa_max = rospy.get_param("/monoped/joint_limits_array/haa_max")
        self.haa_min = rospy.get_param("/monoped/joint_limits_array/haa_min")
        self.hfe_max = rospy.get_param("/monoped/joint_limits_array/hfe_max")
        self.hfe_min = rospy.get_param("/monoped/joint_limits_array/hfe_min")
        self.kfe_max = rospy.get_param("/monoped/joint_limits_array/kfe_max")
        self.kfe_min = rospy.get_param("/monoped/joint_limits_array/kfe_min")

    def _set_init_pose(self):
        """
        Sets the Robot in its init pose
        """
        self.move_joints(self.init_joint)
        return True

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        self._set_init_pose()


    def _set_action(self, action):
        """
        Move the robot based on the action variable given
        """
        joint_states = self.get_joints()
        joint_states_position = joint_states.position
        action_position = [0.0, 0.0, 0.0]
        if action == 0: #Increment haa_joint
            rospy.logdebug("Action Decided:Increment haa_joint>>>")
            action_position[0] = joint_states_position[0] + self.joint_increment_value
            action_position[1] = joint_states_position[1] 
            action_position[2] = joint_states_position[2]
        elif action == 1: #Decrement haa_joint
            rospy.logdebug("Action Decided:Decrement haa_joint>>>")
            action_position[0] = joint_states_position[0] - self.joint_increment_value
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
        elif action == 2: #Increment hfe_joint
            rospy.logdebug("Action Decided:Increment hfe_joint>>>")
            action_position[0] = joint_states_position[0] 
            action_position[1] = joint_states_position[1] + self.joint_increment_value
            action_position[2] = joint_states_position[2]
        elif action == 3: #Decrement hfe_joint
            rospy.logdebug("Action Decided:Decrement hfe_joint>>>")
            action_position[0] = joint_states_position[0] 
            action_position[1] = joint_states_position[1] - self.joint_increment_value
            action_position[2] = joint_states_position[2]
        elif action == 4: #Increment hke_joint
            rospy.logdebug("Action Decided:Increment kfe_joint>>>")
            action_position[0] = joint_states_position[0] 
            action_position[1] = joint_states_position[1] 
            action_position[2] = joint_states_position[2] + self.joint_increment_value
        elif action == 5: #Decrement hke_joint
            rospy.logdebug("Action Decided:Decrement kfe_joint>>>")
            action_position[0] = joint_states_position[0] 
            action_position[1] = joint_states_position[1] 
            action_position[2] = joint_states_position[2] - self.joint_increment_value
        #haa
        action_position[0] = max(min(action_position[0], self.haa_max),self.haa_min)
        #hfe
        action_position[1] = max(min(action_position[1], self.hfe_max),self.hfe_min)
        #kfe
        action_position[2] = max(min(action_position[2], self.kfe_max),self.kfe_min)
        self.move_joints(action_position)

    def _get_obs(self):
        """
        Here we define what sensor data of our robots observations
        """
        distance_from_desired_point = self.get_distance_from_point(self.init_joint)
        base_roll, base_pitch, base_yaw = self.get_base_orientation_euler()
        base_angular_velocity = self.get_base_angular_velocity() 
        base_angular_vel_x = base_angular_velocity.x
        base_angular_vel_y = base_angular_velocity.y
        base_angular_vel_z = base_angular_velocity.z
        base_linear_acceleration = self.get_base_linear_acceleration()
        base_linear_acceleration_x = base_linear_acceleration.x
        base_linear_acceleration_y = base_linear_acceleration.y
        base_linear_acceleration_z = base_linear_acceleration.z
        contact_force = self.get_contact_force()
        joint_states = self.get_joints()

        observations = [distance_from_desired_point, base_roll, base_pitch, base_yaw,
        base_angular_vel_x, base_angular_vel_y, base_angular_vel_z,
        base_linear_acceleration_x, base_linear_acceleration_y, base_linear_acceleration_z, 
        contact_force,
        joint_states.position[0],joint_states.position[1],joint_states.position[2],
        joint_states.effort[0],joint_states.effort[1],joint_states.effort[2]
        ]
        return observations

    def _is_done(self, observations):
        """
        Decide if episode is done based on the observations
        """
        monoped_height_ok = self.monoped_height_ok()
        monoped_orientation_ok = self.monoped_orientation_ok()
        done = not(monoped_height_ok and monoped_orientation_ok)
        return done

    def calculate_reward_joint_position(self, weight):
        """
        We calculate reward base on the joints configuration. The more near 0 the better.
        :return:
        """
        acumulated_joint_pos = 0.0
        joint_states = self.get_joints()
        for joint_pos in joint_states.position:
            acumulated_joint_pos += abs(joint_pos)
            rospy.logdebug("calculate_reward_joint_position>>acumulated_joint_pos=" + str(acumulated_joint_pos))
        reward = weight * acumulated_joint_pos
        rospy.loginfo("calculate_reward_joint_position>>reward=" + str(reward))
        return reward

    def calculate_reward_joint_effort(self, weight):
        """
        We calculate reward base on the joints effort readings. The more near 0 the better.
        :return:
        """
        acumulated_joint_effort = 0.0
        joint_states = self.get_joints()
        for joint_effort in joint_states.effort:
            acumulated_joint_effort += abs(joint_effort)
            rospy.logdebug("calculate_reward_joint_effort>>joint_effort=" + str(joint_effort))
            rospy.logdebug("calculate_reward_joint_effort>>acumulated_joint_effort=" + str(acumulated_joint_effort))
        reward = weight * acumulated_joint_effort
        rospy.loginfo("calculate_reward_joint_effort>>reward=" + str(reward))
        return reward
    
    def calculate_reward_contact_force(self, weight):
        """
        We calculate reward base on the contact force.
        The nearest to the desired contact force the better.
        :return:
        """
        force_magnitude = self.get_contact_force()
        force_displacement = force_magnitude - self.desired_force
        rospy.logdebug("calculate_reward_contact_force>>force_magnitude=" + str(force_magnitude))
        rospy.logdebug("calculate_reward_contact_force>>force_displacement=" + str(force_displacement))
        reward = weight * abs(force_displacement)
        rospy.loginfo("calculate_reward_contact_force>>reward=" + str(reward))
        return reward
    
    def calculate_reward_orientation(self, weight):
        """
        We calculate the reward based on the orientation.
        :return:
        """
        # 0->x 1->y 2->z
        curren_orientation = self.get_base_orientation_euler()
        yaw_displacement = curren_orientation[2] - self.desired_yaw
        rospy.logdebug("calculate_reward_orientation>>[R,P,Y]=" + str(curren_orientation))
        acumulated_orientation_displacement = abs(curren_orientation[0]) + abs(curren_orientation[1]) + abs(yaw_displacement)
        reward = weight * acumulated_orientation_displacement
        rospy.loginfo("calculate_reward_orientation>>reward=" + str(reward))
        return reward
    
    def calculate_reward_distance_from_des_point(self, weight):
        """
        We calculate the distance from the desired point.
        """
        distance = self.get_distance_from_point(self.desired_pose)
        reward = weight * distance
        rospy.loginfo("calculate_reward_orientation>>reward=" + str(reward))
        return reward
        
    def _compute_reward(self, observations, done):
        """
        Return the reward based on the observations given
        """
        if not done:
            r1 = self.calculate_reward_joint_position(self.weight1)
            r2 = self.calculate_reward_joint_effort(self.weight2)
            r3 = self.calculate_reward_contact_force(self.weight3)
            r4 = self.calculate_reward_orientation(self.weight4)
            r5 = self.calculate_reward_distance_from_des_point(self.weight5)
            total_reward = self.alive_reward - r1 - r2 - r3 - r4 - r5
        else:
            total_reward = self.done_reward
        return total_reward
        
    # Internal TaskEnv Methods
    def get_distance_from_point(self, p_end):
        """
        get distance from current position
        """
        a = numpy.array((self.odom.pose.pose.position.x, self.odom.pose.pose.position.y,
         self.odom.pose.pose.position.z))
        b = numpy.array((p_end[0], p_end[1], p_end[2]))
        distance = numpy.linalg.norm(a - b)
        return distance
    
    def get_base_orientation_euler(self):
        """
        convert from quaternions to euler
        """
        orientation_list = [self.odom.pose.pose.orientation.x,
                            self.odom.pose.pose.orientation.y,
                            self.odom.pose.pose.orientation.z,
                            self.odom.pose.pose.orientation.w] 
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        return roll, pitch, yaw

    def get_base_height(self):
        height = self.odom.pose.pose.position.z
        rospy.logdebug("BASE-HEIGHT="+str(height))
        return height

    def get_base_angular_velocity(self):
        base_angular_velocity = self.imu.angular_velocity
        return base_angular_velocity

    def get_base_linear_acceleration(self):
        base_linear_acceleration = self.imu.linear_acceleration
        return base_linear_acceleration

    def get_contact_force(self):
        contact_force = self.contact_force
        contact_force_np = numpy.array((contact_force.x, contact_force.y, contact_force.z))
        force_magnitude = numpy.linalg.norm(contact_force_np)
        return force_magnitude
    
    def monoped_height_ok(self):
        height_ok = self.min_height <= self.get_base_height() < self.max_height
        return height_ok
    
    def monoped_orientation_ok(self):
        roll, pitch, _ = self.get_base_orientation_euler()
        roll_ok = self.max_incl > abs(roll)
        pitch_ok = self.max_incl > abs(pitch)
        orientation_ok = roll_ok and pitch_ok
        return orientation_ok