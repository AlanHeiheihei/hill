#include "dstar/common.h"

//=====================全局使用的变量===================================
ofstream dstar_target_point("/home/deepdriving/shenrk/xc_Project/catkin_car/src/obstacle_runout/output/dstar_target_point.txt");
ofstream new_end_point("/home/deepdriving/shenrk/xc_Project/catkin_car/src/obstacle_runout/output/new_end_point.txt");
ofstream ob_polygon_file("/home/deepdriving/shenrk/xc_Project/catkin_car/src/obstacle_runout/output/ob_polygon.txt");
const string dstar_path_log = "/home/deepdriving/shenrk/xc_Project/catkin_car/src/obstacle_runout/output/dstar_path/";
ofstream curvePath("/home/deepdriving/shenrk/xc_Project/catkin_car/src/obstacle_runout/output/curvePath.txt");
ofstream Dstar_ij_path("/home/deepdriving/shenrk/xc_Project/catkin_car/src/obstacle_runout/output/Dstar_ij_path.txt");

int PlanMode = 1;//默认绕障规划模式 zqq 0801
GPSPoint arbitrary_point;//任意目标点
const double m_per_cell_ = 0.70;//栅格宽度
int map_id_num = 1;//当前所在子图编号
int is_map_change = -1;//是否切图标志位
vector<GPSPoint> all_map;//存储规划路径
volatile bool new_map_have_received = false;//是否有新的规划路径产生
GPSPoint car_real_time_location;//存储小车的实时位置
volatile int MapPath_current_index = 0;//当前位置在规划路径数据下的帧下标
string ALL_MAP("");
vector<GPSPoint> Obstacle;//障碍物点
volatile bool new_grid_map_received = false;//是否收到新的动态栅格图
pthread_rwlock_t newMap_data_rwlock = PTHREAD_RWLOCK_INITIALIZER;//初始化读写锁

//=========================================全局函数======================================================
//以下两个函数用来传递sort第三个参数，对vector进行排序
bool DecreaseSort (GPSPoint a,GPSPoint b) { return (a.y>b.y); }//根据y值降序排序
bool IncreaseSort (GPSPoint a,GPSPoint b) { return (a.TanVal<b.TanVal); }//根据tanval值升序排序

double Min(double a,double b){
    if(a < b){
        return a;
    }
    else return b;
}
double Max(double a,double b){
    if(a > b){
        return a;
    }
    else return b;
}

string getTime()
{
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d--%H:%M:%S",localtime(&timep) );
    return tmp;
}