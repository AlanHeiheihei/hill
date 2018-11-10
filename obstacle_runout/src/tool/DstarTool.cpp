#include "dstar/DstarTool.h"
#include "dstar/B_Spline.h"


LIST_PAIR grid_map;
vector<PATH> start_point;

void show_list(LIST_STATE path){
    LIST_STATE::iterator i=path.begin();
    while(i != path.end()){
        cout<<"("<<i->x<<","<<i->y<<")"<<" ";
        Dstar_ij_path<<"("<<i->x<<","<<i->y<<")"<<" ";
        i++;
    }
    cout<<endl;
    Dstar_ij_path<<endl;
}

/*
    string send_dstar_path(list<GPSPoint> final_path)
    发送dstar路径topic给控制
*/
string send_dstar_path(list<GPSPoint> final_path){
    char buf[128];
    string path_string("");
    list<GPSPoint>::iterator i=final_path.begin();
    string file_time = getTime();
    ofstream dstar_path(dstar_path_log + file_time + ".txt"); 
    while(i != final_path.end())
    {
        sprintf(buf,"%lf %lf %lf %s %s %lf %d %d;",i->x,i->height,i->y,i->cname.c_str(),i->sname.c_str(),i->heading,
                                                                                                    i->key,i->is_turn);
        string tmp("");
	    tmp = buf;
        dstar_path << tmp << endl;
        path_string.append(tmp);
        i++;
    }
    return path_string;
}

/*
    void XYToIJ(double start_x,double start_y,double end_x,double end_y,LIST_PAIR grid_tmp, ipoint2 ij_tmp[])
    找到dstar起始点和终点对应的栅格ij值
*/
void XYToIJ(double start_x,double start_y,double end_x,double end_y,LIST_PAIR grid_tmp, ipoint2 ij_tmp[])
{
    int flag_s = 0,flag_e = 0;
    double half_grid = m_per_cell_/2;
    LIST_PAIR::iterator i=grid_tmp.begin();
    while(i != grid_tmp.end()){
        double x_min = i->first.real_x - half_grid;
        double x_max = i->first.real_x + half_grid;
		double y_min = i->first.real_y - half_grid;
		double y_max = i->first.real_y + half_grid;
        
		if(start_x <= x_max && start_x >= x_min && start_y <= y_max && start_y >= y_min && flag_s < 1){
			ij_tmp[0].x = i->first.x;
            ij_tmp[0].y = i->first.y;
            ij_tmp[0].cost = i->second;
            flag_s++;
		}
        if(end_x <= x_max && end_x >= x_min && end_y <= y_max && end_y >= y_min && flag_e < 1){
			ij_tmp[1].x = i->first.x;
            ij_tmp[1].y = i->first.y;
            ij_tmp[1].cost = i->second;
            flag_e++;
		}
        if(flag_s == 1 && flag_e == 1){
            return;
        }
        i++;
    }
    cout<<"DstarTool.h: cant find the matched ij! "<<flag_s<<" "<<flag_e<<endl;
}

/*
    PATH *calcu_heading(PATH *path,int path_size)
    计算路径上每个点的heading角度值
*/
vector<PATH> calcu_heading(vector<PATH> path,int path_size){
    for(int i=0; i<path_size; i++){
        if(i == path_size-1){
            path[i].heading = path[i-1].heading;  //最后一个点的角度设置
            return path;
        }
        double calcu_tmp = 0;
        double delta_x = path[i+1].x - path[i].x;
        double delta_y = path[i+1].y - path[i].y;
        double theta = atan2(delta_y,delta_x);
        double heading_tmp = theta*180/PI;
        if(delta_x == 0 && delta_y >0){
            path[i].heading = 0;
        }else if(delta_x == 0 && delta_y <0){
            path[i].heading = 179;
        }else if(delta_x >= 0 && delta_y == 0){
            path[i].heading = 90;
        }else if(delta_x < 0 && delta_y == 0){
            path[i].heading = -90;
        }else if(delta_x>0 && delta_y<0)
        {
            path[i].heading = fabs(heading_tmp) + 90;
        }else if(delta_x<0 && delta_y>0)
        {
            path[i].heading = -(heading_tmp - 90);
        }else if(delta_x>0 && delta_y>0)
        {
            path[i].heading = 90 - heading_tmp;
        }else if(delta_x<0 && delta_y<0)
        {
            if(heading_tmp <= -135)
            {
                calcu_tmp = fabs(heading_tmp) - 135;
                path[i].heading = -135 + calcu_tmp;
            }
            else
            {
                calcu_tmp = 135 - fabs(heading_tmp);
                path[i].heading = -135 - calcu_tmp;
            }
        }
    }
}

/*
    list<GPSPoint> create_final_path(PATH *path,int path_size)
    填充路径属性，填充两规划点之间路径点
*/
list<GPSPoint> create_final_path(vector<PATH> path,int path_size){
    list<GPSPoint> final_path;
    final_path.clear();
    GPSPoint path_tmp; 
    for(int i=0; i<path_size-1; i++){
        double delta_x = path[i+1].x - path[i].x;
        double delta_y = path[i+1].y - path[i].y;
        
        for(int j=0; j<DENSITY; j++){
            path_tmp.x = path[i].x + j*delta_x/DENSITY;
            path_tmp.y = path[i].y + j*delta_y/DENSITY;
            path_tmp.heading = path[i].heading;  //角度
            path_tmp.height = 0;
            path_tmp.cname = 'a';
            path_tmp.sname = 'b';
            path_tmp.key = 1;
            path_tmp.is_turn = 0;
            final_path.push_back(path_tmp);
        }
        if(i == path_size-2){
            path_tmp.x = path[i+1].x;
            path_tmp.y = path[i+1].y;
            path_tmp.heading = path[i+1].heading;  //角度
            path_tmp.height = 0;
            path_tmp.cname = 'a';
            path_tmp.sname = 'b';
            path_tmp.key = 1;
            path_tmp.is_turn = 0;
            final_path.push_back(path_tmp);
        }
    }

    return final_path;
}

list<GPSPoint> create_curve_final_path(vector<PATH> path){
    list<GPSPoint> final_path;
    final_path.clear();
    GPSPoint path_tmp; 
    for(int i=0; i<path.size(); i++){
        path_tmp.x = path[i].x;
        path_tmp.y = path[i].y;
        path_tmp.heading = path[i].heading;  //角度
        path_tmp.height = 0;
        path_tmp.cname = 'a';
        path_tmp.sname = 'b';
        path_tmp.key = 1;
        path_tmp.is_turn = 0;
        final_path.push_back(path_tmp);
    }

    return final_path;
}

/*
    vector<PATH> PathCurveSmooth(vector<PATH> control_pts)
    将dstar规划出的几个栅格组成的路径，通过B-Spline曲线方法转换为平滑的曲线
*/
vector<PATH> PathCurveSmooth(vector<PATH> control_pts)
{
    int p = 3;
    int max_control_point_cnt_idx = 0;//max_knots_idx  - p -1;
    int max_knots_idx = 0;
    vector<float> coeffis;//(max_control_point_cnt_idx+1)
    vector<float> knots_arr;
    vector<PATH> final_path;
    int init_knots[] = {0,0,0,0,1,1,1,1};
    max_control_point_cnt_idx = control_pts.size()-1;
    cout<<"max_control_point_cnt_idx: "<<max_control_point_cnt_idx<<endl;

    if(max_control_point_cnt_idx >= 3)
    {
        max_knots_idx = max_control_point_cnt_idx + p +1;
        //为了让得到的曲线，经过首尾控制点，对u0,和um进行p次重复，即u0和um共有p+1个。 knots_size >= 8
        knots_arr.clear();
        for(int i = 0; i < sizeof(init_knots) / sizeof(init_knots[0]); i++){
            knots_arr.push_back(init_knots[i]);
        }
        int delta_knots_num = max_knots_idx + 1 - knots_arr.size();
        cout<<"delta_knots_num: "<<delta_knots_num<<endl;
        if(delta_knots_num > 0){
            float delta_add_num = ((float)((int)(1/((float)delta_knots_num + 1)*100)))/100;//保留两位小数
            cout<<"delta_add_num: "<<delta_add_num<<endl;
            for(int i = 0; i < delta_knots_num; i++){
                knots_arr.insert((knots_arr.begin()+p+1+i),(knots_arr[p+i] + delta_add_num));
            }
        }
        
        for(int i = 0; i < knots_arr.size(); i++){
            cout<< knots_arr[i] << "  ";
        }
        cout << endl;

        int knots_size = knots_arr.size();
        cout<<"max_knots_idx: "<<max_knots_idx<<" knots_size: "<<knots_size<<endl;

        PATH point_tmp;
        for(float u=0;u<1;u=u+0.02)
        {
            calc_b_spline_coefficient(max_control_point_cnt_idx,p,max_knots_idx,knots_arr,u,coeffis);
            //计算
            float u_x=0;
            float u_y=0;
            for(int i=0;i<=max_control_point_cnt_idx;i++){
                u_x+=coeffis[i]*control_pts[i].x;
                u_y+=coeffis[i]*control_pts[i].y;
            }
            point_tmp.x = u_x;
            point_tmp.y = u_y;
            final_path.push_back(point_tmp);
            curvePath<<u<<" "<<u_x<<" "<<u_y<<endl;
        }
    }
    else
    {
        cout<<"Path point size less than 4, can't curve smooth ."<<endl;
    }
    return final_path;
}

/*
    list<GPSPoint> create_path(LIST_STATE mypath, GPSPoint end_point, GPSPoint all_map_current_loca)
    根据dstar生成的初始路径进行处理
    1.根据路径ij值找到相应的xy
    2.将路径起始点设置为pathplan路径上的当前点;终点设置为路径上找到的目标点
    3.路径曲线平滑
    4.计算路径角度
    5.路径信息填充
*/
list<GPSPoint> create_path(LIST_STATE mypath, GPSPoint end_point, GPSPoint all_map_current_loca)
{
    //PATH *path = new PATH[mypath.size()];
    vector<PATH> path;
    vector<PATH> path_2;
    PATH path_tmp;
    mypath.pop_back();  //mypath.size() - 1
    list<GPSPoint> final_path;
    
    LIST_STATE::iterator a = mypath.begin();//1.
    while(a != mypath.end())
    {
        LIST_PAIR::iterator b = grid_map.begin();
        while(b != grid_map.end()){
            if(a->x == b->first.x && a->y == b->first.y)
            {
                path_tmp.x = b->first.real_x;
                path_tmp.y = b->first.real_y;
                path.push_back(path_tmp);
            }
            b++;
        }
        a++;
    }
    path[0].x = all_map_current_loca.x;//2.
    path[0].y = all_map_current_loca.y;
    path_tmp.x = end_point.x;
    path_tmp.y = end_point.y;
    path.push_back(path_tmp); //end point

    path_2 = PathCurveSmooth(path);//3.
    if(path_2.empty())
    {
        path = calcu_heading(path,mypath.size()+1);//4.
        final_path = create_final_path(path,mypath.size()+1);//5.
    }
    else{
        path_2 = calcu_heading(path_2,path_2.size());
        final_path = create_curve_final_path(path_2);
    }

    // path = calcu_heading(path,mypath.size()+1);//3.final_path = calcu_heading(final_path,);
    // final_path = create_final_path(path,mypath.size()+1);//4.
    
    return final_path;
}

/*
    vector<GPSPoint> GetObstacleRange(vector<GPSPoint> Obstacle)
    获取障碍物的范围(多边形)
*/
vector<GPSPoint> GetObstacleRange(vector<GPSPoint> Obstacle)
{
    vector<GPSPoint> ob_polygon;
    vector<GPSPoint> polygon_vertical;//存储x值相等的点
    GPSPoint X_Max_Point;//x值最大的一点
    //init
    X_Max_Point.x = INT_MIN;
    X_Max_Point.y = INT_MAX;
    //1.找x值最大的一点,若存在多个x值最小的点，则取其中y值最小的点
    for(int i = 0; i<Obstacle.size(); i++ ){
        if(Obstacle.at(i).x > X_Max_Point.x){
             X_Max_Point = Obstacle.at(i);
        }          
        else if(Obstacle.at(i).x == X_Max_Point.x){
            if(Obstacle.at(i).y < X_Max_Point.y){
                X_Max_Point = Obstacle.at(i);
            }            
        }
    }
    //2.计算其余点与x值最大的点跟水平线的正切值
    X_Max_Point.TanVal = INT_MIN;//将该点置为第一个点（正切值最小）
    ob_polygon.push_back(X_Max_Point);
    for(int i = 0; i<Obstacle.size(); i++ ){
        if(Obstacle.at(i).x == X_Max_Point.x && Obstacle.at(i).y == X_Max_Point.y){
            continue;
        }
        if(Obstacle.at(i).x == X_Max_Point.x){
            Obstacle.at(i).TanVal = INT_MAX;//垂直,tan值为无限大
            polygon_vertical.push_back(Obstacle.at(i));
        }
        else{
            Obstacle.at(i).TanVal = (Obstacle.at(i).y - X_Max_Point.y)/fabs(Obstacle.at(i).x - X_Max_Point.x);
            ob_polygon.push_back(Obstacle.at(i));        
            }        
        }
    //3.根据正切值升序排序ob_plygon、根据y值降序排序plygon_vertical
    sort(ob_polygon.begin(),ob_polygon.end(),IncreaseSort);
    sort(polygon_vertical.begin(),polygon_vertical.end(),DecreaseSort);
    cout<<"size : "<<polygon_vertical.size()<<endl;
    //4.整合两个vector
    ob_polygon.insert(ob_polygon.end(),polygon_vertical.begin(),polygon_vertical.end());
    return ob_polygon;
}

/*
    bool JudgePointInObstacleRange(GPSPoint point, vector<GPSPoint> ob_polygon)
    判断该点是不是在障碍物外切矩形范围内
*/
bool JudgePointInObstacleRange(GPSPoint point, vector<GPSPoint> ob_polygon)
{
    //由该点往左边画射线，当射线与多边形交点个数为奇数时，则该点在多边形内，若为偶数（包括0）则认为该点不在多边形内
    int nCount = ob_polygon.size();
    int nCross = 0;//交点个数
    double xCross = 0;//交点的x坐标
    GPSPoint start_point_line,end_point_line;
    for(int i = 0; i<nCount; i++){
        start_point_line = ob_polygon.at(i);
        end_point_line = ob_polygon.at((i+1)%nCount);
        if(start_point_line.y == end_point_line.y)
            continue;
        if(point.y < Min(start_point_line.y ,end_point_line.y) || point.y > Max(start_point_line.y ,end_point_line.y))
            continue;
        else {
            //两点确定一条直线方程，算出交点坐标
            xCross = (point.y - start_point_line.y) * (double)(end_point_line.x - start_point_line.x) / (double)(end_point_line.y -start_point_line.y) + start_point_line.x;
            if(xCross > point.x){
                // 只统计p向左射线的交点
                nCross++;//相交                
            }
        }
    }
    //根据奇偶性判断
    return (nCross % 2 == 1);
}

/*
    int GetRealTimeLocationIndexFromMapPath(GPSPoint real_time_location,vector<GPSPoint> MapPath)
    根据路径方向寻找机器人当前索引值current_index（目前没有用）
*/
int GetRealTimeLocationIndexFromMapPath(GPSPoint real_time_location,vector<GPSPoint> MapPath)
{
    int NumOfPointsInPath = MapPath.size();
    int orientation = 0;//记录小车此时大致的方向，分y轴正负，x轴正负四个方向，分别用0 1 2 3 表示

    for(int i = 0; i < NumOfPointsInPath; i++){
        //判断是否是拐点位置
        if(1 == MapPath.at(i).is_turn){
            if(MapPath.at(i).heading > 45 && MapPath.at(i).heading < 135)
                orientation = 2;//x轴正方向
            else if(MapPath.at(i).heading < -45 && MapPath.at(i).heading > -135)
                    orientation = 3;//x轴负方向
                else if(MapPath.at(i).heading > -45 && MapPath.at(i).heading < 45)
                        orientation = 0;//y轴正方向
                    else if((MapPath.at(i).heading > 135 && MapPath.at(i).heading < 180) || (MapPath.at(i).heading > -180 && MapPath.at(i).heading < -135))
                            orientation = 1;//y轴负方向
        }
        switch(orientation){
            case 0 : {
                   if(real_time_location.y < MapPath.at(i).y){
                        cout<<"index: "<<orientation<<" "<<real_time_location.x<<" "<<real_time_location.y<<"  end: "<<MapPath.at(i).x<<" "<<MapPath.at(i).y<<" "<<MapPath.at(i).heading<<endl;
                        return i;
                   }                       
                    else break;
            }
            case 1 : {
                    if(real_time_location.y > MapPath.at(i).y){
                        cout<<"index: "<<orientation<<" "<<real_time_location.x<<" "<<real_time_location.y<<"  end: "<<MapPath.at(i).x<<" "<<MapPath.at(i).y<<" "<<MapPath.at(i).heading<<endl;
                        return i;
                   }                       
                    else break;
            }
            case 2 : {
                    if(real_time_location.x < MapPath.at(i).x){
                        cout<<"index: "<<orientation<<" "<<real_time_location.x<<" "<<real_time_location.y<<"  end: "<<MapPath.at(i).x<<" "<<MapPath.at(i).y<<" "<<MapPath.at(i).heading<<endl;
                        return i;
                   }                       
                    else break;
            }
            case 3 : {
                    if(real_time_location.x > MapPath.at(i).x){
                        cout<<"index: "<<orientation<<" "<<real_time_location.x<<" "<<real_time_location.y<<"  end: "<<MapPath.at(i).x<<" "<<MapPath.at(i).y<<" "<<MapPath.at(i).heading<<endl;
                        return i;
                   }
                    else break;
            }
        }
    }
    return  UNEXIST;
}

/*
    GPSPoint GetNewEndPoint(int &end_point_index,GPSPoint now_location,vector<GPSPoint> Obstacle,vector<GPSPoint> MapPath)
    计算机器人此次绕障路径的终点，任务点为前方障碍物后面2个/1个栅格的点，或为pathplan路径的终点
*/
GPSPoint GetNewEndPoint(int &end_point_index,GPSPoint now_location,vector<GPSPoint> Obstacle,vector<GPSPoint> MapPath)
{
    vector<GPSPoint> ob_polygon = GetObstacleRange(Obstacle);//获取障碍物的范围

    int i = MapPath_current_index;//decoded by zbl on jul 24 小车当前所处位置的索引值，由控制那边发过来；
    cout<<"current_index: "<<MapPath_current_index<<endl;

    double per_point_interval;
    if(MapPath_current_index == 0){
        per_point_interval = 0.02;
    }else{
        per_point_interval = Max(fabs(MapPath.at(MapPath_current_index).x - MapPath.at(MapPath_current_index-1).x),fabs(MapPath.at(MapPath_current_index).y - MapPath.at(MapPath_current_index-1).y)); 
        if(per_point_interval < 0.02)
            per_point_interval = 0.02; 
    }
         
    for(i +=int(m_per_cell_/per_point_interval)*2;i<MapPath.size();i+=int(m_per_cell_/per_point_interval))
    {//往前找不属于障碍物范围内的点
        if(!JudgePointInObstacleRange(MapPath.at(i),ob_polygon))
        {
            if((i+int(m_per_cell_/per_point_interval)*2 < MapPath.size()))
            {
                i+=int(m_per_cell_/per_point_interval)*2;//往后再退2个栅格                
            }
            else
            {
                if((i+int(m_per_cell_/per_point_interval) < MapPath.size()))
                {
                i+=int(m_per_cell_/per_point_interval);//往后再退1个栅格  
                }     
            }
            end_point_index = i;
            cout<<"index_point: "<<MapPath.at(i).x <<" "<<MapPath.at(i).y<<" "<<i<<endl;
            return MapPath.at(i);
        }     
    }

    end_point_index = MapPath.size()-1;
    return MapPath.at(MapPath.size()-1);

}

/*
    ipoint2 GetNextPointCantReach(ipoint2 start_point, GPSPoint real_time_location)
    如果障碍物与机器人在同一栅格，便调用此函数寻找机器人行驶方向前方的栅格，将此栅格置为不可达
*/
ipoint2 GetNextPointCantReach(ipoint2 start_point, GPSPoint real_time_location)
{
    ipoint2 next_point;
    double now_heading = real_time_location.heading;

	int current_i = start_point.x;

	int current_j = start_point.y;

	if(now_heading < 22.5 && now_heading >= -22.5){
            current_j++;
			cout<<"1 "<<"("<<current_i<<","<<current_j<<")."<<endl;
	}
    else if((now_heading > 157.5 && now_heading < 180) || (now_heading < -157.5 && now_heading > -180)){
            current_j--;
			
			cout<<"2"<<"("<<current_i<<","<<current_j<<")."<<endl;
	}
    else if(now_heading <112.5  && now_heading > 67.5){
            current_i--;

			cout<<"3"<<"("<<current_i<<","<<current_j<<")."<<endl;
	}
    else if(now_heading < -67.5 && now_heading > -112.5){
            current_i++;
			
			cout<<"4"<<"("<<current_i<<","<<current_j<<")."<<endl;	
	}else if(now_heading < 67.5 && now_heading > 22.5){
            current_i--;
            current_j++;
			
			cout<<"5"<<"("<<current_i<<","<<current_j<<")."<<endl;
	}else if(now_heading < 157.5 && now_heading > 112.5){
            current_i--;
            current_j--;
			
			cout<<"6"<<"("<<current_i<<","<<current_j<<")."<<endl;
	}else if(now_heading > -67.5 && now_heading < -22.5){
            current_i++;
            current_j++;
			cout<<"7"<<"("<<current_i<<","<<current_j<<")."<<endl;
	}else if(now_heading < -112.5 && now_heading > -157.5){
            current_i++;
            current_j--;
			cout<<"8"<<"("<<current_i<<","<<current_j<<")."<<endl;
	}
    next_point.x = current_i;
    next_point.y = current_j;
    next_point.cost = -1;

    return next_point;
}


//做障碍物外切长方形
// GPSPoint GetNewEndPoint(GPSPoint now_location,vector<GPSPoint> Obstacle,vector<GPSPoint> MapPath){
//     ObstacleRange ob_range = GetObstacleRange(Obstacle);//获取障碍物的范围
//     int i = 0;
//     while(car_real_time_location.y > MapPath.at(i).y){
//         i++;
//     }
//     for(i+30;i<MapPath.size();i++){//往前找不属于障碍物范围内且在规划路径上的点
//     handler_time<<"mappath current："<<i<<" "<<MapPath.at(i).x<<" "<<MapPath.at(i).y<<" "<<MapPath.size()<<endl;
//     cout<<"mappath current："<<i<<" "<<MapPath.at(i).x<<" "<<MapPath.at(i).y<<" "<<MapPath.size()<<endl;
//         if(!JudgePointInObstacleRange(MapPath.at(i),ob_range))
//             return MapPath.at(i);
//     }
//     //int real_time_location_index = GetRealTimeLocationIndexFromMapPath(MapPath);//获取当前小车位置处于

// }
