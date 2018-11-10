#ifndef _READ_H_
#define _READ_H_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cstring>
#include <vector>
#include <string>
#include <sstream>
using namespace std;

#define MAX_NUM 9999999999
#define AHEAD_GRID_COST 6

extern ofstream static_grid_map_log;
extern ofstream local_obstacle_log;
extern const string startfile;
extern const string dynamic_gridmap_log_path;
extern const string pgm_path;
extern const string yaml_path;
extern const string obstacle_log_path;

typedef struct point_msg{
    double x;
    double y;
	double heading;
}Point;

typedef struct{
    int flag;             //相当于dstar中的cost值，每个栅格的代价值
    int occupied;         //每个栅格中被占用的像素点个数
    double grid_x,grid_y; //每个栅格中心点坐标
    int i,j;              //每个栅格在PGM图中的ij值
}GRID;

extern vector<GRID> static_grid_map;
extern vector<GRID> dynamic_grid_map;
extern vector<Point> global_obspoint;

extern const double occupied_treshold;
extern bool obstacle_runout_received;
extern vector<Point> start_point;
extern double current_heading;
extern int map_id_num;
extern int is_map_change;

extern double m_per_cell_;
extern int gray_threshold;
extern double theta;

extern int row;
extern int col;
extern double pix;
extern int grid_row;
extern int grid_col;
extern int pix_per_grid_num;
extern int grid_num;

string getTime();
void open_map_spoint();

#endif