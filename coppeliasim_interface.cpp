//coppeliasim_interface.cpp
#include <coppeliasim_interface.h>
#include <iostream>
#include <stdio.h>
#include <windows.h>

#include <thread>

void CoppeliaSimInterface::_load_parameters() {
	FILE* fp = NULL;
	char parameter_name[10];
	char value[20];
	fp = fopen(file_path, "r");
	if (fp == NULL) {
		std::cout << "" << std::endl;
		std::cout << "(Client.load_parameters)" << std::endl;
		std::cout << "     parameters.txt was not found" << std::endl;
		std::cout << "     check the path in client.h " << std::endl;
		std::cout << "     this program continues with default parameters " << std::endl;
		//return -1;
	}
	else {

		while (fscanf(fp, "%s %s", &parameter_name, &value) != EOF)
		{

			if (strcmp(parameter_name, "simIP") == 0)  strcpy(simIP, value);
			if (strcmp(parameter_name, "PortNumber") == 0)  PortNumber = atoi(value);

			if (strcmp(parameter_name, "grid_rx") == 0)  grids_enabled_[0] = atoi(value);
			if (strcmp(parameter_name, "grid_ry") == 0)  grids_enabled_[1] = atoi(value);
			if (strcmp(parameter_name, "grid_rz") == 0)  grids_enabled_[2] = atoi(value);
			if (strcmp(parameter_name, "grid_lx") == 0)  grids_enabled_[3] = atoi(value);
			if (strcmp(parameter_name, "grid_ly") == 0)  grids_enabled_[4] = atoi(value);
			if (strcmp(parameter_name, "grid_lz") == 0)  grids_enabled_[5] = atoi(value);

			//if (strcmp(parameter_name, "color_R") == 0)  points_color_[0] = atoi(value);
			//if (strcmp(parameter_name, "color_G") == 0)  points_color_[1] = atoi(value);
			//if (strcmp(parameter_name, "color_B") == 0)  points_color_[2] = atoi(value);


		}
		//std::cout << simIP << std::endl;
		//std::cout << PortNumber << std::endl;

		fclose(fp);
		std::cout << "coppeliasim_interface: parameters loaded" << std::endl;
	}
}

void CoppeliaSimInterface::_save_parameters() {
	FILE* fp = NULL;
	fp = fopen(file_path, "w");
	if (fp == NULL) {
		std::cout << "" << std::endl;
		std::cout << "(Client.save_parameters)" << std::endl;
		std::cout << "     parameters.txt was not found" << std::endl;
		std::cout << "     saving parameters failed " << std::endl;
		std::cout << "     check the path in client.h " << std::endl;
	}
	
	fprintf(fp, "simIP %s\n", simIP); //<- doesnt work well
	fprintf(fp, "PortNumber %d\n", PortNumber);

	fprintf(fp, "grid_rx %d\n", grids_enabled_[0]);
	fprintf(fp, "grid_ry %d\n", grids_enabled_[1]);
	fprintf(fp, "grid_rz %d\n", grids_enabled_[2]);
	fprintf(fp, "grid_lx %d\n", grids_enabled_[3]);
	fprintf(fp, "grid_ly %d\n", grids_enabled_[4]);
	fprintf(fp, "grid_lz %d\n", grids_enabled_[5]);

	//fprintf(fp, "color_R %d\n", points_color_[0]);
	//fprintf(fp, "color_G %d\n", points_color_[1]);
	//fprintf(fp, "color_B %d\n", points_color_[2]);
	
	fclose(fp);
}


CoppeliaSimInterface::CoppeliaSimInterface()
	//for communicate LabView
	:labview_interface_(new LabViewInterface("192.168.1.100", 6430))
{
	std::cout<< "coppeliasim_interface: constructing" <<std::endl;
	file_path = "src/pointpkg/coppeliasim_interface_parameters.txt";
	CoppeliaSimInterface::_load_parameters();
	std::cout << "" << std::endl;
	std::cout << "(CoppeliaSimInterface.initialize())" << std::endl;
	std::cout << "     connecting to coppeliasim..." << std::endl;
	std::cout << "     please start the simulation " << std::endl;
	while (ClientID == -1) {
		std::cout << "     ClientID  " << ClientID << std::endl;
		ClientID = simxStart(simIP, PortNumber, true, true, 2000, 5);
	}
	std::cout << "     !!  connected  !!" << std::endl;

	simxGetObjectHandle(ClientID, "ParallelLink_Tip_Reference", &ReferenceHandle, simx_opmode_blocking);


	simxGetObjectHandle(ClientID, "ToolColor", &ToolHandle, simx_opmode_blocking);


	//for pcl::remover, get Tip Pose
	simxGetObjectHandle(ClientID, "RemoverTip", &RemoverHandle, simx_opmode_blocking);
	simxGetObjectHandle(ClientID, "ParallelLink_Tip0", &LTipHandle, simx_opmode_blocking);

	std::cout << "coppeliasim_interface: constructed" << std::endl;
}

CoppeliaSimInterface::~CoppeliaSimInterface() {
	simxFinish(ClientID);
	CoppeliaSimInterface::_save_parameters();
	labview_interface_->~LabViewInterface();
}

void CoppeliaSimInterface::update(int& number_of_points, std::vector<float>& points, std::vector<int>& color, int option_function = COP_FUNC_MAIN) {


	std::thread t1([&]() {
		_show_cloud(number_of_points, points, color, option_function);
	});

	std::thread t2([&]() {
		labview_interface_->update();
		_set_tip_pose();
	});

	std::thread t3([&]() {
		_get_tip_pose();
	});

	t3.join();
	t2.join();
	t1.join();



	//_getPointsFromSim(number_of_points, points, color);
	//_set_tool_color();

	//std::cout << "cop update" << std::endl;


	++counter;

}

void CoppeliaSimInterface::_show_cloud(int number_of_points, std::vector<float>& points, std::vector<int>& color, int option_function = COP_FUNC_MAIN) {
	int n = 3 * number_of_points;
	int n_c = 3 * number_of_points;
	if (points.size()==0) {
		n = 0;
		//std::cout << "cop points nullptr " << std::endl;
	}
	if (color.size()==0) {
		n_c = 0;	
		//std::cout << "cop color nullptr " << std::endl;
	}

	if (n > 0) {
		if (option_function == COP_FUNC_MAIN) {

			//double a = clock();
			simxCallScriptFunction(ClientID, "Point_Cloud_Main", sim_scripttype_childscript, "ShowPointCloud", n_c, &color[0], n, &points[0], 0, NULL, 0, NULL, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);
			//std::cout<< "coppeliasim   "<<double(clock()-a)/1000 <<std::endl;

		}
		else if (option_function == COP_FUNC_APPEARED) {
			//simxCallScriptFunction(ClientID, "PointClouds", sim_scripttype_childscript, "ShowAppearedPointCloud", 0, NULL, n, &points[0], 0, NULL, 0, NULL, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);
		}
	}
}

void CoppeliaSimInterface::_get_tip_pose() {
	
	simxGetObjectPosition(ClientID, LTipHandle, -1, &L_tip_pose[0], simx_opmode_oneshot);
	simxGetObjectOrientation(ClientID, LTipHandle, -1, &L_tip_pose[3], simx_opmode_oneshot);
	//for (int i = 0; i < 7; i++) std::cout << L_tip_pose[i] << " ";
	//std::cout << std::endl;

	simxGetObjectPosition(ClientID, RemoverHandle, -1, &L_remover_pose[0], simx_opmode_oneshot);
	simxGetObjectOrientation(ClientID, RemoverHandle, -1, &L_remover_pose[3], simx_opmode_oneshot);

}

void CoppeliaSimInterface::_getPointsFromSim(int& number_of_points, std::vector<float>& points, std::vector<int>& color) {
	int n_points, n_color;
	int* color_;
	float* points_;
	simxCallScriptFunction(ClientID, "tool", sim_scripttype_childscript, "getPoints", 0, NULL, 0, NULL, 0, NULL, 0, NULL, &n_color, &color_, &n_points, &points_, NULL, NULL, NULL, NULL, simx_opmode_blocking);
	std::cout << "coppeliasim" << std::endl;
	//std::cout << points[0] << std::endl;
	int CNT = 0;
	points.clear();
	color.clear();
	for (int i = 0; i < n_points; i++) { 
		points.push_back(points_[i]);
		color.push_back(color_[i%3]);
	}
	//number_of_points = 1;// n_points / 3;
	number_of_points = n_points / 3;

	std::cout << points.size() << std::endl;
}

void CoppeliaSimInterface::_set_tool_color() {
	float position[3] = { Params::tool_color_x, 0., 0. };
	simxSetObjectPosition(ClientID, ToolHandle, -1, position, simx_opmode_oneshot);
}


void CoppeliaSimInterface::_set_tip_pose() {
	std::vector<float> vf = labview_interface_->get_tip_pose();

	if (vf[3] == 0.0 && vf[4] == 0.0){
		std::cout << "labview_interface::_set_tip_pose: " << "TCP 0.0 error" << std::endl;
		return;
	}
	simxSetObjectPosition(ClientID, LTipHandle, -1, &vf[0], simx_opmode_oneshot);
	//std::cout << vf[2] << std::endl;

	float tmp[3];
	//simxSetObjectOrientation(ClientID, LTipHandle, -1, tmp, simx_opmode_oneshot);
	//simxSetObjectOrientation(ClientID, LTipHandle, -1, tmp, simx_opmode_oneshot);

	//if (!(vf[3] == 0.0 && vf[4] == 0.0)) simxSetObjectOrientation(ClientID, LTipHandle, ReferenceHandle, &vf[3], simx_opmode_oneshot);
	//simxSetObjectQuaternion(ClientID, LTipHandle, -1, &vf[3], simx_opmode_oneshot);
	//simxSetObject
}


/*
void CoppeliaSimInterface::update_RGB(int number_of_points, float* points, int* color) {
	if (number_of_points > 0) {
		int n = 3 * number_of_points;
		simxCallScriptFunction(ClientID, "PointClouds", sim_scripttype_childscript, "ShowPointCloudRGB", n, &color[0], n, &points[0], 0, NULL, 0, NULL, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);
	}
}
*/

bool CoppeliaSimInterface::check_loop() {
	if (simxGetConnectionId(ClientID) == -1) return false;
	return true;
}

std::vector<float> CoppeliaSimInterface::get_L_tip_transform() {
	std::vector<float> ret(7);
	for (int i = 0; i < 7; i++) ret[i] = L_tip_pose[i];
	return ret;
}

std::vector<float> CoppeliaSimInterface::get_remover_transform() {
	std::vector<float> ret(7);
	for (int i = 0; i < 7; i++) ret[i] = L_remover_pose[i];
	return ret;
}
