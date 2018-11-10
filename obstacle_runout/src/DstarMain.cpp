#include "dstar/DstarTool.h"
#include "dstar/SubscribeTopicHandler.h"
#include "limits.h"


int main(int argc,char **argv) 
{
    ros::init(argc, argv, "dstar");
    ros::NodeHandle n;
    
    topic_exchange_dstar(n);
    
    ros::Rate rate(50);
    bool status = ros::ok();

    while(status)
    {
        ros::spinOnce();
        
        //获取了read_map发送过来的动态栅格图 和 pathplan规划的路径
        if(new_grid_map_received && new_map_have_received)
        {
            printf("receiving!!!!!!!\n");
            Dstar *dstar = new Dstar();
            LIST_STATE mypath;
            list<GPSPoint> final_path;
            int start_i,start_j,end_i,end_j,end_point_index;
            GPSPoint end_point;
            ipoint2 start_end_point[2];
            ipoint2 next_point;
            next_point.cost = 1;
            
            /*
            **1.绕障模式，自动寻找任务点
            **2.任意点到达模式，由外部指定任务点
            */
            switch(PlanMode)
            {
                case ObstacleRunout://1
                {
                    end_point = GetNewEndPoint(end_point_index,car_real_time_location,Obstacle,all_map);
                    cout<<"ObstacleRunout Mode,auto plan the end point."<<" end_x:"<<end_point.x<<" "<<end_point.y<<endl;
                    break;
                }

                case ArbitraryPointReach://2
                {
                    end_point = arbitrary_point;
                    PlanMode = 1;
                    cout<<"ArbitraryPointReach Mode, receive end point from ctrl."<<endl;
                    break;
                }
            }

            //end_point = GetNewEndPoint(end_point_index,car_real_time_location,Obstacle,all_map);
            //输出找到的新的终点
            new_end_point<<MapPath_current_index<<" "<<end_point.x<<" "<<end_point.y<<" "<<car_real_time_location.x
                                <<" "<<car_real_time_location.y<<" "<<end_point_index<<endl;
            
            XYToIJ(all_map[MapPath_current_index].x, all_map[MapPath_current_index].y, end_point.x, 
                                                                end_point.y, grid_map,start_end_point);
           
            start_i = start_end_point[0].x;
            start_j = start_end_point[0].y;
            end_i = start_end_point[1].x;
            end_j = start_end_point[1].y;

            //如果障碍物与机器人在同一个栅格上，则把机器人前方栅格置为不可达
            if(start_end_point[0].cost == -1)
            {
                next_point = GetNextPointCantReach(start_end_point[0], car_real_time_location);
            }
            //如果障碍物在pathplan路径的最后一个点上，则给控制发送无法规划的topic，机器人停下等待，直到障碍物移开
            if(start_end_point[1].cost == -1)
            {
                cout<<"DstarMain.cpp: end point can't reach,send to lidarctrl."<<endl;
                std_msgs::String msg;
                msg.data = "can't reach";
                pub_dstar_path.publish(msg);
                new_grid_map_received = false;
                continue;
            }
            
            //set start and goal point
            dstar->init(start_i,start_j,end_i,end_j); 
            cout<<"start_x:"<<all_map[MapPath_current_index].x<<" start_y:"<<all_map[MapPath_current_index].y
                                                            <<" end_x:"<<end_point.x<<" "<<end_point.y<<endl;
            cout<<start_i<<" "<<start_j<<" "<<end_i<<" "<<end_j<<endl;

            //updateCell()输入所有栅格信息给dstar用于规划
            LIST_PAIR::iterator i = grid_map.begin();
            while(i != grid_map.end())
            {
                if((i->first.x == start_i && i->first.y == start_j))
                {
                    i++;
                    continue;                    
                }
                dstar->updateCell(i->first.x, i->first.y, i->second);
                i++;
            }
            cout<<"next_point.cost: "<<next_point.cost<<endl;
            dstar->updateCell(next_point.x, next_point.y, next_point.cost);
            
            //replan()规划dstar路径，规划失败则发0给控制
            if(!dstar->replan())
            {
                cout<<"DstarMain.cpp: plan path failed,send 0 to lidarctrl."<<endl;
                std_msgs::String msg;
                msg.data = "0";
                pub_dstar_path.publish(msg);
                new_grid_map_received = false;
                continue;
            }
            //getPath()获取规划的路径
            mypath = dstar->getPath();
            show_list(mypath); 
            if(mypath.size()<=1){
                cout<<"DstarMain.cpp: path too short,send to lidarctrl."<<endl;
                std_msgs::String msg;
                msg.data = "can't reach";
                pub_dstar_path.publish(msg);
                new_grid_map_received = false;
                continue;
            }
            cout<<"create path!"<<endl;
            //对路经进行处理
            final_path = create_path(mypath, end_point, all_map[MapPath_current_index]);
            cout<<"send_dstar_path!!"<<endl;
            string path_tmp = send_dstar_path(final_path);
            //在路径末尾加上一个终点的索引值
            path_tmp.append("/");
            path_tmp.append(std::to_string(end_point_index));
            cout<<"send finish"<<endl;
            std_msgs::String msg;
            msg.data = path_tmp;
            pub_dstar_path.publish(msg);

            new_grid_map_received = false;
        }

        rate.sleep();
    }
    return 0;
}
