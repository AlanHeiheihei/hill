#include "read_map/obstacleAdd.h"

Point real_time_location;
int replan_received = -1;

/*
	vector<Point> LocalToGlobalObsPoint(vector<Point> obstacle_point,int pointCount)
	将得到的障碍物点坐标从以小车中心为原点坐标系转换为全局坐标系
*/
vector<Point> obstacleAdd::LocalToGlobalObsPoint(vector<Point> obstacle_point,int pointCount)
{
	vector<Point> global_obspoint_tmp;
	string file_time = getTime();
	std::ofstream global_obstacle_log(obstacle_log_path + file_time + ".txt");
	for(int i=0; i<pointCount; i++){
		Point global_tmp;
		double heading = real_time_location.heading;
		global_tmp.x = obstacle_point.at(i).x*cos(-heading) - obstacle_point.at(i).y*sin(-heading)+real_time_location.x;
		global_tmp.y = obstacle_point.at(i).x*sin(-heading) + obstacle_point.at(i).y*cos(-heading)+real_time_location.y;
		global_obspoint_tmp.push_back(global_tmp);
		global_obstacle_log<<setprecision(8)<<global_obspoint_tmp.at(i).x<<" "<<global_obspoint_tmp.at(i).y<<" "
		<<real_time_location.x<<" "<<real_time_location.y<<" "<<real_time_location.heading<<endl;
	}
	return global_obspoint_tmp;
}

/*
	int PointInGrid(double x,double y)
	找到坐标为（x，y）的点对应的栅格下标
*/
int obstacleAdd::PointInGrid(double x,double y)
{
	for(int i=0; i<grid_num; i++){
		double half_grid = m_per_cell_/2;
		double x_min = static_grid_map.at(i).grid_x - half_grid;
		double x_max = static_grid_map.at(i).grid_x + half_grid;
		double y_min = static_grid_map.at(i).grid_y - half_grid;
		double y_max = static_grid_map.at(i).grid_y + half_grid;
		if(x <= x_max && x >= x_min && y <= y_max && y >= y_min){
			return i;
		}
	}
	cout<<"obstacleAdd: cant find the matched grid! "<<x << " " << y << endl;
	return 0;
}

/*
	void AddUnreachableGridAfterRobot()
	判断机器人在遇障停下时，当前位置的前一个栅格是否被占用，如果没有便将flag值设为AHEAD_GRID_COST(此值大于2)，
	这样在传给dstar规划时，规划的绕障路径不会往前直行到前面的栅格，然后再90度旋转绕障，而是直接向斜前方45度角绕障。
*/
void obstacleAdd::AddUnreachableGridAfterRobot()
{
	double now_heading = real_time_location.heading;
	int current_num = PointInGrid(real_time_location.x,real_time_location.y);
	int current_j = (current_num+1)%(grid_row)-1;
	if(current_j == -1) current_j = grid_row-1;
	int current_i = floor((current_num+1)/(grid_row));
	int occupied_num = pix_per_grid_num*pix_per_grid_num;
	if(now_heading < 22.5 && now_heading >= -22.5){
		cout<<"1 obstacleAdd.h: the grid is "<<"("<<current_i<<","<<current_j<<"). next flag: "<<dynamic_grid_map.at(current_i*grid_row+current_j+1).flag<<endl;
		if(current_j < grid_row && dynamic_grid_map.at(current_i*grid_row+current_j+1).flag == 1){
			dynamic_grid_map.at(current_i*grid_row+current_j+1).flag = AHEAD_GRID_COST;
			cout<<"1 obstacleAdd.h: the grid after robot is "<<"("<<current_i<<","<<current_j+1<<")."<<endl;
		}
	}else if((now_heading > 157.5 && now_heading < 180) || (now_heading < -157.5 && now_heading > -180)){
		cout<<"1 obstacleAdd.h: the grid is "<<"("<<current_i<<","<<current_j<<"). next flag: "<<dynamic_grid_map.at(current_i*grid_row+current_j-1).flag<<endl;
		if(current_j > 0 && dynamic_grid_map.at(current_i*grid_row+current_j-1).flag == 1){
			dynamic_grid_map.at(current_i*grid_row+current_j-1).flag = AHEAD_GRID_COST;
			cout<<"2 obstacleAdd.h: the grid after robot is "<<"("<<current_i<<","<<current_j-1<<")."<<endl;
		}
	}else if(now_heading <112.5  && now_heading > 67.5){
		cout<<"1 obstacleAdd.h: the grid is "<<"("<<current_i<<","<<current_j<<"). next flag: "<<dynamic_grid_map.at((current_i-1)*grid_row+current_j).flag<<endl;
		if(current_i > 0 && dynamic_grid_map.at((current_i-1)*grid_row+current_j).flag == 1){
			dynamic_grid_map.at((current_i-1)*grid_row+current_j).flag = AHEAD_GRID_COST;
			cout<<"3 obstacleAdd.h: the grid after robot is "<<"("<<current_i-1<<","<<current_j<<")."<<endl;
		}	
	}else if(now_heading < -67.5 && now_heading > -112.5){
		cout<<"1 obstacleAdd.h: the grid is "<<"("<<current_i<<","<<current_j<<"). next flag: "<<dynamic_grid_map.at((current_i+1)*grid_row+current_j).flag<<endl;
		if(current_i < grid_col && dynamic_grid_map.at((current_i+1)*grid_row+current_j).flag == 1){
			dynamic_grid_map.at((current_i+1)*grid_row+current_j).flag = AHEAD_GRID_COST;
			cout<<"4 obstacleAdd.h: the grid after robot is "<<"("<<current_i+1<<","<<current_j<<")."<<endl;
		}		
	}else if(now_heading < 67.5 && now_heading > 22.5){
		cout<<"1 obstacleAdd.h: the grid is "<<"("<<current_i<<","<<current_j<<"). next flag: "<<dynamic_grid_map.at((current_i-1)*grid_row+current_j+1).flag<<endl;
		if(current_i > 0 && current_j < grid_row && 
													dynamic_grid_map.at((current_i-1)*grid_row+current_j+1).flag == 1){
			dynamic_grid_map.at((current_i-1)*grid_row+current_j+1).flag = AHEAD_GRID_COST;
			cout<<"5 obstacleAdd.h: the grid after robot is "<<"("<<current_i-1<<","<<current_j+1<<")."<<endl;
		}
	}else if(now_heading < 157.5 && now_heading > 112.5){
		cout<<"1 obstacleAdd.h: the grid is "<<"("<<current_i<<","<<current_j<<"). next flag: "<<dynamic_grid_map.at((current_i-1)*grid_row+current_j-1).flag<<endl;
		if(current_i > 0 && current_j > 0 && 
												dynamic_grid_map.at((current_i-1)*grid_row+current_j-1).flag == 1){
			dynamic_grid_map.at((current_i-1)*grid_row+current_j-1).flag = AHEAD_GRID_COST;
			cout<<"6 obstacleAdd.h: the grid after robot is "<<"("<<current_i-1<<","<<current_j-1<<")."<<endl;
		}
	}else if(now_heading > -67.5 && now_heading < -22.5){
		cout<<"1 obstacleAdd.h: the grid is "<<"("<<current_i<<","<<current_j<<"). next flag: "<<dynamic_grid_map.at((current_i + 1)*grid_row+current_j + 1).flag<<endl;
		if(current_i < grid_col && current_j <grid_row && 
															dynamic_grid_map.at((current_i + 1)*grid_row+current_j + 1).flag == 1){
			dynamic_grid_map.at((current_i + 1)*grid_row+current_j + 1).flag = AHEAD_GRID_COST;
			cout<<"7 obstacleAdd.h: the grid after robot is "<<"("<<current_i + 1<<","<<current_j + 1<<")."<<endl;
		}		
	}else if(now_heading < -112.5 && now_heading > -157.5){
		cout<<"1 obstacleAdd.h: the grid is "<<"("<<current_i<<","<<current_j<<"). next flag: "<<dynamic_grid_map.at((current_i + 1)*grid_row + current_j - 1).flag<<endl;
		if(current_i < grid_col && current_j > 0 &&
													 dynamic_grid_map.at((current_i + 1)*grid_row + current_j - 1).flag == 1){
			dynamic_grid_map.at((current_i + 1)*grid_row+current_j - 1).flag = AHEAD_GRID_COST;
			cout<<"8 obstacleAdd.h: the grid after robot is "<<"("<<current_i+1<<","<<current_j - 1<<")."<<endl;
		}
	}
}

/*
	void dynamicObstacleAdd(vector<Point> global_obspoint,int cloudSize)
	找到障碍物坐标点对应的栅格，并增加对应栅格occupied值
*/
void obstacleAdd::dynamicObstacleAdd(vector<Point> global_obspoint,int cloudSize)
{
	int last_num = -1;
	dynamic_grid_map.clear();
	dynamic_grid_map.assign(static_grid_map.begin(),static_grid_map.end());
	for(int i=0; i<cloudSize; i++)
	{
		int num = PointInGrid(global_obspoint.at(i).x,global_obspoint.at(i).y);
		if(static_grid_map.at(num).occupied < (pix_per_grid_num*pix_per_grid_num))
		{
			dynamic_grid_map.at(num).occupied ++;
		}
		/*if(dynamic_grid_map.at(num).occupied > occupied_treshold
			 && last_num != num){//避免计算多次  /(m_per_cell_*m_per_cell_)
			//EnlargeObstacleRange(dynamic_grid_map[num]);
			AddAfterGrid(dynamic_grid_map.at(num));  //zqq 0702
		}
		last_num = num;*/
	}
	//AddGridAfterRobot();  //zqq 0705
}

/*
	void GetObstacles(const std_msgs::String::ConstPtr &msg)
	从检测障碍物节点的topic获取激光雷达扫到的障碍物点信息，并进行坐标转换和加入动态栅格地图中
*/
void obstacleAdd::GetObstacles(const std_msgs::String::ConstPtr &msg){
	vector<Point> obstacle_point;
	global_obspoint.clear();

	stringstream ss(msg->data);
	string tmp_s;
	Point point_tmp;
	while(getline(ss,tmp_s,';'))
	{
		if(tmp_s.empty()){
			continue;
		}
		
		stringstream final_s(tmp_s);
		final_s>>setprecision(12)>>point_tmp.x>>point_tmp.y;
		obstacle_point.push_back(point_tmp);

		local_obstacle_log<<point_tmp.x<<" "<<point_tmp.y<<endl;
	}

	int pointCount = obstacle_point.size();

	global_obspoint = LocalToGlobalObsPoint(obstacle_point,pointCount);   // 先转换为全局的
	dynamicObstacleAdd(global_obspoint,pointCount);
}

void obstacleAdd::FinalDataHandler(const std_msgs::String::ConstPtr &msg){
	double real_time_x = 0,real_time_y = 0,real_time_heading = 0;
	string tmp = msg->data;
	stringstream stringin(tmp);
	stringin>>setprecision(10)>>real_time_x>>real_time_y>>real_time_heading;//heading 为弧度

	real_time_location.x = real_time_x;
	real_time_location.y = real_time_y;
	real_time_location.heading = real_time_heading;
	
}

void obstacleAdd::HeadingHandler(const std_msgs::String::ConstPtr &msg){
	string tmp = msg->data;
	stringstream stringin(tmp);
	stringin>>current_heading;
}

void obstacleAdd::ReplanHandler(const std_msgs::Int8::ConstPtr &msg){
    replan_received = msg->data; //1是不规划，0是规划
}

void obstacleAdd::MapIdHandler(const std_msgs::Int8::ConstPtr &msg){    //zqq 0704
	map_id_num = msg->data;
    //map_id_num = 1;
    if(map_id_num <= 0)
        cout<<"createGrid.h: map_id_num is "<<map_id_num<<" ,its less than 1!"<<endl;
}

void obstacleAdd::MapChangeHandler(const std_msgs::Int8::ConstPtr &msg){ //0711
	is_map_change = msg->data;
}

void obstacleAdd::topic_exchange(ros::NodeHandle n){
    obstacle_point_sub = n.subscribe<std_msgs::String>("obstacle_out",1028,&obstacleAdd::GetObstacles,this);
    subfinal = n.subscribe<std_msgs::String>("/final_data",1000,&obstacleAdd::FinalDataHandler,this);
    dstar_replan_sub = n.subscribe<std_msgs::Int8>("dstar_plan",5,&obstacleAdd::ReplanHandler,this);
    current_heading_sub = n.subscribe<std_msgs::String>("heading_send",5,&obstacleAdd::HeadingHandler,this);
    map_id_sub = n.subscribe<std_msgs::Int8>("map_index",5,&obstacleAdd::MapIdHandler,this);
    map_change_sub = n.subscribe<std_msgs::Int8>("map_change",5,&obstacleAdd::MapChangeHandler,this);
    dynamic_map = n.advertise<std_msgs::String>("dynamic_grid_map",1000); 
    obstacle_point_pub = n.advertise<std_msgs::String>("obstacle_points",1000);
    printf("!@!!!!!!\n");
}


/*void EnlargeObstacleRange(GRID grid){
	int occupied_num = pix_per_grid_num*pix_per_grid_num;
	//cout<<"obstacle enlarge:"<<grid.i<<" "<<grid.j<<" "<<grid_col<<endl;
	if(grid.j < grid_row){
		dynamic_grid_map.at(grid.i*grid_row+(grid.j+1)).occupied = occupied_num;
		//cout<<"1: "<<dynamic_grid_map[grid.i*(grid.j+1)].grid_x<<" "<<dynamic_grid_map[grid.i*(grid.j+1)].grid_y<<endl;
		if(grid.i <grid_col){
			dynamic_grid_map.at((grid.i+1)*grid_row+(grid.j+1)).occupied = occupied_num;
		}
	}
	if(grid.i <grid_col){
		dynamic_grid_map.at((grid.i+1)*grid_row+grid.j).occupied = occupied_num;
		if(grid.j > 0){
			dynamic_grid_map.at((grid.i+1)*grid_row+(grid.j-1)).occupied = occupied_num;
		}
	}
	if(grid.j > 0){
		dynamic_grid_map.at(grid.i*grid_row+(grid.j-1)).occupied = occupied_num;
		if(grid.i > 0){
			dynamic_grid_map.at((grid.i-1)*grid_row+(grid.j-1)).occupied = occupied_num;
		}
	}
	if(grid.i > 0)
		dynamic_grid_map.at((grid.i-1)*grid_row+grid.j).occupied = occupied_num;
		if(grid.j < grid_row){
			dynamic_grid_map.at((grid.i-1)*grid_row+(grid.j+1)).occupied = occupied_num;
		}
}

void AddGridAfterObs(GRID grid){
	int occupied_num = pix_per_grid_num*pix_per_grid_num;
	if(current_heading < 15 && current_heading > -15){
		if(grid.j < grid_row)
			dynamic_grid_map.at(grid.i*grid_row+grid.j+1).occupied = occupied_num;
	}else if(current_heading > 165 && current_heading < -165){
		if(grid.j > 0)
			dynamic_grid_map.at(grid.i*grid_row+grid.j-1).occupied = occupied_num;
	}else if(current_heading < 60 && current_heading > 30){
		if(grid.i > 0)
			dynamic_grid_map.at((grid.i-1)*grid_row+grid.j).occupied = occupied_num;
	}else if(current_heading < -30 && current_heading > -60){
		if(grid.i < grid_col)
			dynamic_grid_map.at((grid.i+1)*grid_row+grid.j).occupied = occupied_num;
	}
}*/