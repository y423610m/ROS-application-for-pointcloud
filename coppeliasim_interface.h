//coppeliasim_interface.h
#pragma once
#ifndef COP_FUNC_MAIN
#define COP_FUNC_MAIN 0
#endif

#ifndef COP_FUNC_APPEARED
#define COP_FUNC_APPEARED 1
#endif

#ifndef COP_FUNC_DISAPPEARED
#define COP_FUNC_DISAPPEARED 2
#endif

#include <point_cloud_shower.h>
#include <params.h>

extern "C" {
#include "extApi.h"
}

#include "labview_interface.h"
#include <memory>

class CoppeliaSimInterface{
private:
	char* file_path;


	int ClientID = -1;
	int PortNumber = 19998;
	simxChar simIP[20] = "127.0.0.1";
	int counter = 0;
	bool grids_enabled_[6] = { true, true, true, true, true, true };
	int inInts_[10];

	//_tip_position
	int ReferenceHandle;
	int RemoverHandle=-1;
	int RTipHandle=-1;
	int LTipHandle = -1;
	float R_remover_pose[7];// init pcd data tip pose
	float L_remover_pose[7];
	float R_tip_pose[7];// real time robot arm tip pose
	float L_tip_pose[7];
	//float R_tip_orientation[4], L_tip_orientation[4];
	
	//_set_tool_color
	int ToolHandle;


	//communicate with LabView
	std::unique_ptr<LabViewInterface> labview_interface_;

	void _load_parameters();
	void _save_parameters();
	void _show_cloud(int number_of_points, std::vector<float>& points, std::vector<int>& color, int option_function);
	void _get_tip_pose();
	void _getPointsFromSim(int& number_of_points, std::vector<float>& points, std::vector<int>& color);
	void _set_tool_color();
	void _set_tip_pose();


public:
	CoppeliaSimInterface();
	~CoppeliaSimInterface();
	void update(int& number_of_points, std::vector<float>& points, std::vector<int>& color, int option_function);
	//void update_RGB(int number_of_points, float* points,int* color);
	bool check_loop();

	bool* get_grids_enabled(int n) { return &grids_enabled_[n]; }
	std::vector<float> get_L_tip_transform();//m, rad
	std::vector<float> get_remover_transform();//m, rad
	//float* get_R_tip_position(){ return R_tip_position; }
	//float* get_L_tip_position(){ return L_tip_position; }
	//float* get_R_tip_orientation(){ return R_tip_orientation; }
	//float* get_L_tip_orientation(){ return L_tip_orientation; }
};

