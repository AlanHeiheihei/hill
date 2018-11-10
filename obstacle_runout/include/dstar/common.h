#ifndef _COMMON_H_
#define _COMMON_H_

#include <iostream>
#include <fstream> //ifstream
#include <string> //compare
#include <vector>
using namespace std;

#define ObstacleRunout 1
#define ArbitraryPointReach 2

struct GPSPoint{
    double lat;
    double lon;

    double x;
    double y;
    double height;

    double GaussX;
    double GaussY;

    double LX=0;
    double LY=0;

    double heading; 

    int map_index;
    int is_finish_turn;

    int map_change;

    int is_turn;
    int key;

    string cname;
    string sname;

    double TanVal;
};

extern int PlanMode;
extern GPSPoint arbitrary_point;
extern ofstream dstar_target_point;
extern ofstream new_end_point;
extern ofstream ob_polygon_file;
extern ofstream curvePath;
extern ofstream Dstar_ij_path;
extern const string dstar_path_log;

extern const double m_per_cell_;
extern int map_id_num;
extern int is_map_change;
extern vector<GPSPoint> all_map;
extern volatile bool new_map_have_received;
extern GPSPoint car_real_time_location;
extern volatile int MapPath_current_index;
extern string ALL_MAP;
extern vector<GPSPoint> Obstacle;
extern volatile bool new_grid_map_received;

//=========================================全局函数======================================================
//以下两个函数用来传递sort第三个参数，对vector进行排序
bool DecreaseSort (GPSPoint ,GPSPoint );
bool IncreaseSort (GPSPoint ,GPSPoint );

double Min(double ,double );

double Max(double ,double );

string getTime();

#endif