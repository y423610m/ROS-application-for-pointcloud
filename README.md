# ROS-application-for-pointcloud

Abstruct:
this program get point cloud data from depth camera through rostopic, process(detect object, apply filter or something) and show it in simulation(CoppeliaSim) or new window. run
          
          rosrun pointpkg pointer






ros_interface:    get pointcloud data from 3D cameras via ros 

pcl:    class for processing pointcloud using PCL(point cloud library)

coppeliasim_interface:    send pointcloud data to 3D simulation

operator_side_manafer:    integrate and manage these 3 classes

gui:    class to chage parameters for processing with gui and mouce controler

labview_interface:    communicate with labview in TCP protocol

tdmaker: class to create artificial time delay


****** this program requires some independencies below *******

opencv

cv-ui

PCL(point cloud library)



****** please look into CMakeList.txt for detail ******
