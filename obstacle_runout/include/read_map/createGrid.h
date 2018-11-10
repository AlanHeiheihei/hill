#ifndef _CREATEGRID_H_
#define _CREATEGRID_H_

#include "read_map/read.h"
#include "read_map/readMap.h"
#include "read_map/readYaml.h"
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"

extern IMG *newMAP;
extern YAML *newYAML;

class createGrid
{
    public:
        ros::NodeHandle nh;
        ros::Subscriber final_data_sub;
        void read_pgm_and_yaml();
        GRID LocalToGlobal(GRID grid_tmp);
        vector<GRID> PGMToOriginMap(vector<GRID> grid_map,YAML *newYAML,int grid_num);
        vector<GRID> OriginMapToGlobalMap(vector<GRID> grid_map,int grid_num);
        vector<GRID> createStaticGrid();
        ReadMap readmap;
        ReadYaml readyaml;
        

};

#endif

