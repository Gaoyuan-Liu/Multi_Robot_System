# Panda ERG 

## Start with position control
i) make an executable file with a subscriber and pusblish the reference joint angles 
ii) currently launch with two terminals: one terminal for launching the gazebo controller (panda_gazebo package) and one terminal for running the executable file (panda_erg)

## Make your own PID controller and use effort control (v1)
i) there is no gravity compensation, so to let it work you have to turn off the gravity in gazebo
ii) gravity compensation should be possible with I gain? 

## Make your own PID controller and use effort control (v2)
To add gravity compensation we will use the KDL library. 
i) convert .urdf.xacro file to .urdf file (look to .urdf.xacro file in panda_description) by runnin the following command in a terminal with sourced workspace
rosrun xacro xacro --inorder -o model.urdf model.urdf.xacro
ii) create KDL tree
https://p4sser8y-words.readthedocs.io/ROS/orocos_kdl.html#usage
Problem with undefined reference solved by 2nd answer on https://answers.ros.org/question/76231/kdl-undefined-reference-to-kdlchain/ 
iii) create KDL chain


## Make testlib 
Be careful with CMakeLists.txt
