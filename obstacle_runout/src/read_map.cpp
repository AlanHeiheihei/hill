#include "read_map/obstacleAdd.h"

/*
    void send_dynamic_map(vector<GRID> dynamic_grid_map, obstacleAdd obstacleadd)
    以string形式发送动态栅格图到Dstar
*/
void send_dynamic_map(vector<GRID> dynamic_grid_map, obstacleAdd obstacleadd)
{
    char buf[128];
    std_msgs::String msg;
    string map_data("");
    for(int a=0; a<grid_num; a++){
        double real_x = dynamic_grid_map.at(a).grid_x;
        double real_y = dynamic_grid_map.at(a).grid_y;
        int i = dynamic_grid_map.at(a).i;
        int j = dynamic_grid_map.at(a).j;
        int flag = dynamic_grid_map.at(a).flag;
        sprintf(buf,"%lf %lf %d %d %d,",real_x,real_y,flag,i,j);
        string tmp("");
	    tmp = buf;
        map_data.append(tmp);
    }
    msg.data = map_data;
    obstacleadd.dynamic_map.publish(msg);
    printf("read_map.cpp: send dynamic map to Dstar.\n");
}

/*
    void send_obstacle_point(obstacleAdd obstacleadd)
    将经过坐标转换的障碍物点信息发给Dstar
*/
void send_obstacle_point(obstacleAdd obstacleadd)
{
    char buf[128];
    string point_send("");
    vector<Point>::iterator i=global_obspoint.begin();
    while(i != global_obspoint.end()){
        string tmp("");
        sprintf(buf,"%lf %lf,",i->x,i->y);
        tmp = buf;
        point_send.append(tmp);
        i++;
    }
    std_msgs::String msg;
    msg.data = point_send;
    obstacleadd.obstacle_point_pub.publish(msg);
}

/*
    void InitGrid(createGrid creategrid)
    读取pgm和yaml文件，创建静态栅格地图，初始化动态栅格图
*/
void InitGrid(createGrid creategrid)
{
    creategrid.read_pgm_and_yaml();
    cout<<"read_map.cpp : recreate static grid!!!"<<endl;
    static_grid_map = creategrid.createStaticGrid();   //静态栅格
    dynamic_grid_map.clear();
    dynamic_grid_map.assign(static_grid_map.begin(),static_grid_map.end());
}

/*
    主函数
*/
int main(int argc,char **argv)
{
    ros::init(argc, argv, "read_map");
    ros::NodeHandle n;

    open_map_spoint();
    obstacleAdd obstacleadd;
    createGrid creategrid;
    InitGrid(creategrid);
    
    obstacleadd.topic_exchange(n);
    ros::Rate rate(100);
    bool status = ros::ok();

    while(status)
    {
        ros::spinOnce();

        //切图时，更新相应子图的栅格图
        if(is_map_change == 1)
	    {
            InitGrid(creategrid);
            is_map_change = -1;
        }

        //收到了从控制发来的绕障规划topic
        if(replan_received == 0)
        {
            cout<<"read_map.cpp: beginning!!!"<<endl;
            //计算每个栅格是否被占用，-1为被占用
            for(int i=0; i<grid_num; i++)
            {
                if(dynamic_grid_map.at(i).occupied > occupied_treshold)
                {
                    dynamic_grid_map.at(i).flag = -1;
                }
                else{
                    dynamic_grid_map.at(i).flag = 1;
                }
            }

            obstacleadd.AddUnreachableGridAfterRobot();  //zqq 0711
            ros::Duration(1).sleep();

            send_dynamic_map(dynamic_grid_map, obstacleadd);
            send_obstacle_point(obstacleadd);

            //log记录静态与动态栅格图数据
            string file_time = getTime();
            std::ofstream dynamic_grid_map_log(dynamic_gridmap_log_path+file_time+".txt");
            for(int i=0; i<grid_num; i++){
                static_grid_map_log<<std::setprecision(6)<<static_grid_map.at(i).grid_x<<" "<<
                                    static_grid_map.at(i).grid_y<<" "<<static_grid_map.at(i).occupied<<endl;
                dynamic_grid_map_log<<std::setprecision(6)<<dynamic_grid_map.at(i).grid_x<<" "<<
                                    dynamic_grid_map.at(i).grid_y<<" "<<dynamic_grid_map.at(i).flag
                                    <<" "<<dynamic_grid_map.at(i).i<<" "<<dynamic_grid_map.at(i).j<<endl;
            }
            replan_received = 1;
        }
        rate.sleep();
    }
    return 0;
}
