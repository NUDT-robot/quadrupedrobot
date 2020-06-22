/*
 * Author : Chang Xu
 * This is a configure file. Every term will be used in quadruped robot
 * to influence calculation result.
*/

//定义该宏将启动关于GAZEBO仿真环境的默认配置，如果使用实物机器人则应取消这个宏定义。
#define ROBOTENV_GAZEBO

// If it is defined, graph data analysis, for example gnuplot data file, will be generated.
#define GRAPH_ANALYSIS

// INS_STEPTIME 应该被设置为惯性导航系统(Inertial Navigation System)传输数据周期。
#define INS_STEPTIME 0.005

// POSITION_STEPFACTOR 用来计算关节角速度，默认值为1000，应该被设置为两个相邻位置信息之间时间的倒数。速度=（当前位置-上一时刻位置）/ POSITION_STEPFACTOR
#ifdef ROBOTENV_GAZEBO
	#define POSITION_STEPFACTOR 1000

	//Gazebo模型中与模型匹配的连杆长度，从身体到脚方向连杆依次标号为1～3
	#define LINK1_LEN 0.0595	//连杆1长度
	#define LINK2_LEN 0.270		//连杆2长度
	#define LINK3_LEN 0.335		//连杆3长度

#else
	#define POSITION_STEPFACTOR 

	//配置真实机器人匹配的连杆长度，从身体到脚方向连杆依次标号为1～3
	#define LINK1_LEN 	//连杆1长度
	#define LINK2_LEN 		//连杆2长度
	#define LINK3_LEN 		//连杆3长度
#endif

