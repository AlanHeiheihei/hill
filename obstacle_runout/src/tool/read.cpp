#include "read_map/read.h"


std::ofstream static_grid_map_log("/home/deepdriving/shenrk/xc_Project/catkin_car/src/obstacle_runout/output/static_grid_map.txt");
std::ofstream local_obstacle_log("/home/deepdriving/shenrk/xc_Project/catkin_car/src/obstacle_runout/output/local_obstacle.txt");
const string startfile("/home/deepdriving/shenrk/xc_Project/map/yingno/map_spoint.txt");
const string dynamic_gridmap_log_path = "/home/deepdriving/shenrk/xc_Project/catkin_car/src/obstacle_runout/output/dynamic_grid_map/";
const string pgm_path = "/home/deepdriving/shenrk/xc_Project/map/yingno/data/";
const string yaml_path = "/home/deepdriving/shenrk/xc_Project/map/yingno/data/";
const string obstacle_log_path = "/home/deepdriving/shenrk/xc_Project/catkin_car/src/obstacle_runout/output/global_obstacle/";

vector<GRID> static_grid_map;//静态栅格图
vector<GRID> dynamic_grid_map;//动态栅格图
vector<Point> global_obspoint; //全局障碍物点

const double occupied_treshold = 0;//判断栅格是否被占用的阈值
bool obstacle_runout_received = true; //判断ctrl是否传出绕障计算消息
vector<Point> start_point;//记录切图起始点坐标
double current_heading = 0;//当前pathplan路径索引的角度
int map_id_num = 1;
int is_map_change = -1;

double m_per_cell_ = 0.70;//每个正方形栅格宽度
int gray_threshold = 128;//像素点灰度阈值
double theta = 0;  //PGM图需要调整的角度


int row = 0;//pgm图长和宽
int col = 0;
double pix = 0;//像素宽度，一般为0.05m
int grid_row = 0;//每行的栅格个数
int grid_col = 0;//每列的栅格个数
int pix_per_grid_num = 0;//每个栅格一行中像素点个数
int grid_num = 0;//栅格总数目

string getTime()
{
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d--%H:%M:%S",localtime(&timep) );
    return tmp;
}

void open_map_spoint()
{
    ifstream infile;
    infile.open(startfile.c_str());
    start_point.clear();
    stringstream ss;
    string string_tmp;
    
    double start_x = 0;
    double start_y = 0;
    double start_heading = 0;

    while(!infile.eof())
    {
        string_tmp.clear();
        ss.clear();
        getline(infile,string_tmp);
        if(string_tmp.compare("") == 0) continue;
        ss<<string_tmp;
        ss>>setprecision(12)>>start_x>>start_y>>start_heading;
        Point point_tmp;
        point_tmp.x = start_x;
        point_tmp.y = start_y;
        point_tmp.heading = start_heading;
        start_point.push_back(point_tmp);
    }
    infile.close();
}