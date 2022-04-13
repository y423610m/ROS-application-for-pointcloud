#include <iostream>
#include <string>
#include <math.h>
#include <time.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//#include <pointer.h>
#include <config.h>
#include <not_used/manager.h>
#include <not_used/client.h>
#include <Params.h>


#include <operator_side_manager.h>
#include <gui.h>


extern "C" {
#include "extApi.h"
}
///////////////////////////////////////
#include <stdio.h>
#include <sstream>
#include <stdlib.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
//


int main(int argc, char** argv) {
	
	ros::init(argc, argv, "pointer");
	ros::NodeHandle nh;

	std::unique_ptr<OperatorSideManager> operator_side_manager(new OperatorSideManager(nh));
	std::unique_ptr<GUI> gui(new GUI(operator_side_manager.get()));

	const int FPS = 5;
	ros::Rate rate(FPS);
	while (operator_side_manager->check_loop()){// && gui->check_loop()) {

		operator_side_manager->update();
		gui->update();

		ros::spinOnce();
		rate.sleep();
	}
	
    // Manager manager(nh);
    // manager.loop();
     
	std::cout << "program ended successfully" << std::endl;

   return 0;
}



///////////////////////////////////////
