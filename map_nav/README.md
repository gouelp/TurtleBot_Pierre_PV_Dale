Turtlebot self inspection project. 

This project is in 3 parts which are more or less independant from each other. 

For the map coverage part and the robot moving part (potential field): 
	Directory "map_nav" with in it the files : 
		- robot_cov.launch : launches the potential field and the decision algorithm
		- mapping_robot.launch : launches the gmapping package with specific parameters for the physical turtlebot
		- mapping.launch : launches the gmapping package with specific parameters for the simulation

		- cover_map.py : the file containing the code that send informations to the robot-control node and takes the decision to begin and end the covering.
		- potential_field.py : the field that manage the robot control. Allows to control the robot from a point to another using potential field or do other controls as 360° rotation

		- msg folder : contains all the needed messages to comunicate with the others nodes of the project
		- srv folder : contains the rosservice message for the communication between the potential_filed node and the cover_map node

For the sign_detection_part and the pose_publisher : 
	Directory "sign_detection_part" with in it the files : 
		- find_object_3d.launch : launches the object detection and recognition and find_object package
		- sign_detection.launch : launched by find_object_3d.launch
		- mapping.launcg : launches the gmapping package with specific parameters for the simulation
		- pose_publisher.py : need to be rosrun, republish the corrected pose of the robot from the tf /map /odom
		- sign_recognition.py : program that recognise the alive and dead workers and send osign information tothe system through /sign_list (launched by find_object_3d.launch)

		- msg folder : contains all the needed messages to comunicate with the others nodes of the project
		- session folder : contains the session parameters for find_object package
		- objects folder : contains the signs for the find_object package

For the costMap and plot:
	Directory ¨costMap-Plot-Files" with in it the files :

		- costMap.py : used for creating the Dijkstra cost map
		- plot.py :    used for plotting the 2D grid

launch files in folder launch :
		- costmap_launch_file1.launch: for launching costMap.py, also calls plot.py

msg folder - contains the message files for :
		- Dijkstra.msg
		- Map.plot.msg
		- sign_message.msg
Ressources : 
		- icones folder : contains all the icones of the detected signs to be plotted in the map  

		
