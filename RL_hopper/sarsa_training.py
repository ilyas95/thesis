#!/usr/bin/env python


import gym
import time
import numpy
import os
import random
import time
import sarsa
from gym import wrappers
import rospy
import rospkg
import hopper_stay_up_env


if __name__ == '__main__':
    rospy.init_node('hopper_gym', anonymous=True, log_level=rospy.INFO)
    # Create the Gym environment
    env = gym.make('HopperStayUp-v0')
    rospy.loginfo ( "Gym environment done")  
    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('hopper_training')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True) 
    rospy.loginfo ( "Monitor Wrapper started")
    last_time_steps = numpy.ndarray(0)
    # Loads parameters from the ROS param server
    Alpha = rospy.get_param("/monoped/alpha")
    Epsilon = rospy.get_param("/monoped/epsilon")
    Gamma = rospy.get_param("/monoped/gamma")
    epsilon_discount = rospy.get_param("/monoped/epsilon_discount")
    nepisodes = rospy.get_param("/monoped/nepisodes")
    nsteps = rospy.get_param("/monoped/nsteps")
    running_step = rospy.get_param("/monoped/running_step")
    # Initialises the algorithm that we are going to use for learning
    sarsa = sarsa.Sarsa(actions=range(env.action_space.n),
                    alpha=Alpha, gamma=Gamma, epsilon=Epsilon)
    initial_epsilon = sarsa.epsilon
    start_time = time.time()
    highest_reward = 0
    # Check if it already exists a training policy, and load it if so
    qfile = "sarsa_states.npy"
    if (os.path.exists(qfile)):
        print("Loading from file:",qfile)
        qlearn.loadQ(qfile)
    # Starts the main training loop: the one about the episodes to do
    for x in range(nepisodes):        
        rospy.logdebug("############### START EPISODE => " + str(x))
        cumulated_reward = 0  
        done = False
        if sarsa.epsilon > 0.05:
            sarsa.epsilon *= epsilon_discount
        # Initialize the environment and get first state of the robot
        observation = env.reset()
        state = ''.join(map(str, observation))
        # Show on screen the actual situation of the robot
        # for each episode, we test the robot for nsteps
        for i in range(nsteps):  
            rospy.loginfo("############### Start Step => "+str(i))
            # Pick an action based on the current state
            action = sarsa.chooseAction(state)
            rospy.loginfo ("Next action is: %d", action)
            # Execute the action in the environment and get feedback
            observation, reward, done, info = env.step(action)
            rospy.loginfo(str(observation) + " " + str(reward))
            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward
            nextState = ''.join(map(str, observation))
            nextAction = sarsa.chooseAction(nextState)
            # Make the algorithm learn based on the results
            sarsa.learn(state, action, reward, nextState, nextAction)
            if not(done):
                state = nextState
            else:
                rospy.loginfo ("DONE")
                last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                break
            rospy.loginfo("############### END Step =>" + str(i))
        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        rospy.logwarn ( ("EP: "+str(x+1)+" - [alpha: "+str(round(sarsa.alpha,2))+" - gamma: "+str(round(sarsa.gamma,2))+" - epsilon: "+str(round(sarsa.epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s)))
    rospy.loginfo ( ("\n|"+str(nepisodes)+"|"+str(sarsa.alpha)+"|"+str(sarsa.gamma)+"|"+str(initial_epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |"))
    l = last_time_steps.tolist()
    l.sort()
    rospy.loginfo("Overall score: {:0.2f}".format(last_time_steps.mean()))
    rospy.loginfo("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))
    #Save the trained policy
    qlearn.save(qfile)
    env.close()