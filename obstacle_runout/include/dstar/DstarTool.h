#ifndef _DSTARTOOL_H
#define _DSTARTOOL_H

#include "dstar/Dstar.h"
#include "common.h"
#include <ros/ros.h>
#include "std_msgs/String.h"

#define UNEXIST INT_MAX

struct CoordinatePoint{
    double x;
    double y;
};

struct ObstacleRange{
    CoordinatePoint left_top,right_top,left_down,right_down;
};

extern LIST_PAIR grid_map;
extern vector<PATH> start_point;


void show_list(LIST_STATE );

string send_dstar_path(list<GPSPoint> );

void XYToIJ(double ,double ,double ,double ,LIST_PAIR , ipoint2 []);

PATH *calcu_heading(PATH *,int );

list<GPSPoint> create_final_path(PATH *,int );

list<GPSPoint> create_path(LIST_STATE , GPSPoint ,GPSPoint );

vector<GPSPoint> GetObstacleRange(vector<GPSPoint> );

bool JudgePointInObstacleRange(GPSPoint , vector<GPSPoint> );

int GetRealTimeLocationIndexFromMapPath(GPSPoint ,vector<GPSPoint> );

GPSPoint GetNewEndPoint(int &,GPSPoint ,vector<GPSPoint> ,vector<GPSPoint> );

ipoint2 GetNextPointCantReach(ipoint2 , GPSPoint );

#endif