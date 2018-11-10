#ifndef _SUBTOPICHANDLER_H_
#define _SUBTOPICHANDLER_H_

#include "common.h"
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include <pthread.h>
#include <sstream>
using namespace std;


ros::Subscriber sub_dynamic_map;
ros::Subscriber sub_final_data;
ros::Publisher pub_dstar_path;
ros::Subscriber sub_current_index;
ros::Subscriber sub_map_data;
ros::Subscriber sub_obstacle_point;
ros::Subscriber map_id_sub;
ros::Subscriber map_change_sub;
ros::Subscriber arbitrary_point_sub;

//=====================全局函数=======================================
void decodePath(string paths,vector<GPSPoint>& map){
  if(paths.find_last_of('/') != -1){
    paths.erase(paths.find_last_of('/'), paths.length() - paths.find_last_of('/'));//zbl 删掉标志位 
    //cout<<"path22222:"<<paths<<endl;  
  }
  stringstream ss(paths);
  string tmp_s;
  while(getline(ss,tmp_s,';')){
    GPSPoint gps;
    stringstream final_s(tmp_s);
    final_s>>setprecision(12)>>gps.x>>gps.height>>gps.y>>gps.cname>>gps.sname>>gps.heading>>gps.key>>gps.is_turn>>gps.map_change;
    gps.GaussX = gps.x;
    gps.GaussY = gps.y;
    //gps.heading = gps.heading * 180.0 / 3.14159265;  
    gps.is_finish_turn = 0;  
    map.push_back(gps);
  }
  cout<<"the path size: "<<map.size()<<endl;
}
void map_Handler(const std_msgs::String::ConstPtr &msg){

  ALL_MAP.clear();
  all_map.clear();
  ALL_MAP = msg->data;
  cout<<"path have receive! ctrl"<<endl;
  decodePath(ALL_MAP,all_map);//解析规划路径
  new_map_have_received = true;

  if(!all_map.empty()){

    cout<<"map size: "<<all_map.size()<<endl;

    cout<<"init angle: "<<all_map[0].heading<<endl;

    for(int i=0;i<all_map.size();i++)     //dyq  0112
    {
      all_map[i].is_turn = 0;
       all_map[i].heading = all_map[i].heading * 57.29; 
    }
    for(int i=1;i<all_map.size();i++)
    {
      if(fabs(all_map[i].heading - all_map[i-1].heading) > 70 && fabs(all_map[i].heading - all_map[i-1].heading) < 290)
      {
        all_map[i].is_turn = 1;
      }
    }

    all_map[0].is_turn = 1;
    all_map[all_map.size()-1].is_turn = 1;
  }
}

void final_data_Handler(const std_msgs::String::ConstPtr &msg){
    stringstream stringin(msg->data);
    stringin>>setprecision(12)>>car_real_time_location.x>>car_real_time_location.y>>car_real_time_location.heading;
}

void current_index_Handler(const std_msgs::Int64::ConstPtr &msg){
    MapPath_current_index = msg->data;
    //handler_time<<"current_index"<<" "<< ros::Time::now().toSec()<<" "<<MapPath_current_index<<endl;
}

void DynamicMapHandler(const std_msgs::String::ConstPtr &msg){
    string MAP_TMP("");
    MAP_TMP = msg->data;
    //cout<<"map:"<<MAP_TMP<<endl;
    stringstream ss(MAP_TMP);
    string string_tmp;
    printf("!!!!!!!\n");
    grid_map.clear();
    while(getline(ss,string_tmp,',')){
        pair<ipoint2,double> grid_tmp;
        
        stringstream final_s(string_tmp);

        final_s>>setprecision(12)>>grid_tmp.first.real_x>>grid_tmp.first.real_y>>grid_tmp.second>>grid_tmp.first.x>>grid_tmp.first.y;
        
        //cout<<"map: "<<grid_tmp.first.real_x<<" "<<grid_tmp.first.real_y<<" "<<grid_tmp.second<<" "<<grid_tmp.first.x<<" "<<grid_tmp.first.y<<endl;
        grid_map.push_back(grid_tmp);
        string_tmp.clear();
    }
    new_grid_map_received = true;
    cout<<"SubscribeTopicHandler.h: received dynamic map..."<<endl;
}

void obstacle_points_Handler(const std_msgs::String::ConstPtr &msg){
    string POINT_TMP("");
    POINT_TMP = msg->data;
    stringstream ss(POINT_TMP);
    string string_tmp;

    while(getline(ss,string_tmp,',')){
        GPSPoint point_tmp;
        
        stringstream final_s(string_tmp);

        final_s>>setprecision(12)>>point_tmp.x>>point_tmp.y;
        //cout<<"obstacle_point:"<<point_tmp.x<<" "<<point_tmp.y<<endl;
        Obstacle.push_back(point_tmp);
        string_tmp.clear();
    }
    cout<<"SubscribeTopicHandler.h: received obstacle points..."<<endl;
}

void MapIdHandler(const std_msgs::Int8::ConstPtr &msg){    //zqq 0704
	map_id_num = msg->data;
}

void MapChangeHandler(const std_msgs::Int8::ConstPtr &msg){
	is_map_change = msg->data;
}

void ArbitraryPointHandler(const std_msgs::String::ConstPtr &msg){
  string POINT_TMP("");
  POINT_TMP = msg->data;
  stringstream ss(POINT_TMP);
  ss>>setprecision(10)>>arbitrary_point.x>>arbitrary_point.y;
  PlanMode = 2;
  cout<<"SubscribeTopicHandler.h: received arbitrary point..."<<endl;
}

void topic_exchange_dstar(ros::NodeHandle n){
    //printf("!22222!\n");
    sub_dynamic_map = n.subscribe<std_msgs::String>("dynamic_grid_map",1000,DynamicMapHandler); 
    pub_dstar_path = n.advertise<std_msgs::String>("dstar_path",1000);
    sub_final_data = n.subscribe<std_msgs::String>("/final_data",1000,final_data_Handler);
    sub_current_index = n.subscribe<std_msgs::Int64>("index_send",5,current_index_Handler);
    sub_map_data = n.subscribe<std_msgs::String>("/map_send",1000,map_Handler);
    sub_obstacle_point = n.subscribe<std_msgs::String>("obstacle_points",1000,obstacle_points_Handler);
    map_id_sub = n.subscribe<std_msgs::Int8>("map_index",5,MapIdHandler);
	  map_change_sub = n.subscribe<std_msgs::Int8>("map_change",5,MapChangeHandler);
    arbitrary_point_sub = n.subscribe<std_msgs::String>("XXXXXXXXXX",10,ArbitraryPointHandler);
}

#endif