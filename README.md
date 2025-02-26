Download Link : https://programming.engineering/product/robotics-in-a-nutshell-12-points/

# Robotics-in-a-Nutshell-12-Points-
Robotics in a Nutshell [12 Points]
Name, Surname, ID Number

Problem 1.1 Robotics in a Nutshell [12 Points]

You are considering to buy a new multi-purpose robot platform. Its kinematic chain has two rotational qf1,3g and two linear qf2,4g degrees of freedom (DoFs), as shown in the figure below. These four joints are actuated with forces and torques of ui , i 2 f1, 2, 3, 4g. A gripper is mounted on the end of the robot, indicated by the letter E. The robot’s base is mounted on a table. We assume that the base Cartesian coordinates at the mount are xbase = [0, 0, 0].

q1 q3

E

q2 q4

                        L 1
                        	

                        Z
                        	

                        y
                        		

                        X

Side View

    Forward Kinematics [2 Points]

Compute the kinematic transformation in the global coordinate system from the base xbase to the end-eﬀector E. Write the solution for the xend-eﬀ = [x, y, z]T according to the joint values qi , where i 2 f1, 2, 3, 4g.

    Inverse Kinematics [2 Points]

Define briefly in your own words the inverse kinematics problem in robotics. Can we always accurately model the inverse kinematics of a robot with a function?

Name, Surname, ID Number

    Diﬀerential Kinematics [4 Points]

Compute the Jacobian matrix J (q) of the robot such that x˙ = J (q) q˙, where q˙ is the first time derivatives of the state vector q of the robot. Explain in a sentence the physical meaning of the Jacobian.

    Singularities [3 Points]

What is the kinematic singularity in robotics? How can you detect it? When does our robotic arm, which was defined above, enter a kinematic singularity?

    Workspace [1 Points]

If your task is to sort items placed on a table, would you buy this robot? Briefly justify your answer.

Name, Surname, ID Number

Problem 1.2 Control [31 Points]

In robotic locomotion it is common to abstract from the robot by using inverted pendulum models. In this exercise we will use a planar double inverted pendulum to test diﬀerent control strategies. Our robot can be controlled by specifying the torque u = [u1, u2] of its motors. Consider that in mechanical systems the torque u is a function of the joint positions q, velocities q˙ and accelerations q¨, as given by

    = M (q) q¨ + c (q, q˙) + g (q) ,

where M denotes the inertial matrix, c (q, q˙) the Coriolis and centripetal forces, and g the gravity terms. In the following exercises assume that these terms are given.

For the programming exercises you will use the attached code. We provide skeletons for controlling the system either in joint space (my_ctl.py) or in task space (my_taskSpace_ctl.py) and a basic functionality for plotting. You can invoke either mode by running jointCtlComp.py or taskCtlComp.py respectively. Attach a printout with plots and a snippet of your source code for each programming exercise.

    PID Controller [2 Points]

What is the form of a proportional-integral-derivative (PID) controller and how could you use it to control a robot, i.e. what physical quantities could you control? Name one positive and one negative aspect of PID controllers.

Name, Surname, ID Number

    Gravity Compensation and Inverse Dynamics Control [4 Points]

Suppose that you would like to create a control law to set the joint angles on the double inverted pendulum model by controlling the torque of the motors. Write a feedback control law which additionally gravity compensates and then extend it to full inverse dynamics control.

Name, Surname, ID Number

    Comparison of Diﬀerent Control Strategies [12 Points]

In the following exercise you will investigate the diﬀerences of the following control algorithms, P, PID, PD with gravity compensation, and full inverse dynamics. The double pendulum is initiated hanging down, with state qstart = [ , 0]. We simulate the system with a time-step d t = 0.002 seconds using symplectic Euler integration and run the simulation for tend = 3s.

Implement the control laws by filling the skeleton file my_ctl.py. Use the following feedback gains KP = 60, KD = 10, KI = 0.1 for the first joint and KP = 30, KD = 6, KI = 0.1 for the second one. The target state of the double

pendulum is set to qdes = [ =2, 0].

Create (max. 4) plots that compare the diﬀerent control strategies and analyze the results. It is your choice how to illustrate your results. In your analysis you should include a discussion on the overall performance of each controller. Which controllers manage to go to the desired point, and how does the choice of a controller aﬀects the behavior of the second joint of the pendulum? Additionally discuss which controller you would choose and why. The provided code is able to generate plot but feel free to modify it if you like. Points will be deducted for confusing plots. Do not forget to include your source code in your solutions.

Name, Surname, ID Number

    Tracking Trajectories [4 Points]

Repeat the same experiment but this time use the provided time-varying target trajectory. Create (max 4) plots that compare the diﬀerent control strategies and analyze the results. In your analysis discuss the overall performance and which controllers track the desired trajectory nicely. Additionally discuss which controller you would choose and why.

    Tracking Trajectories — High Gains [4 Points]

Repeat the same experiment (using the provided trajectory) but this time multiply the gains by ten. Create plots that compare the diﬀerent control strategies and analyze the results. In your analysis discuss the overall performance and compare it to the previous case. Are there any drawbacks of using high gains?

Name, Surname, ID Number

f) Task Space Control [5 Points]

The robot must now reach a desired position in task space xend = [ 0.35, 1.5]. In class we derived the Jacobian transpose, Jacobian pseudo-inverse, and Jacobian pseudo-inverse with damping methods. All of them are im-plemented in my_taskSpace_ctl.py. You are asked to implement also the null-space task prioritization method with a null-space resting posture q = [0, ]. Run the simulation and plot the initial and final configuration of the robot. Then, change the resting posture to q = [0, ] and redo the plots. Analyze in a couple of sentences your observation. Use the same damping coeﬃcient 10 6 and include a code snippet to your solutions.

7
