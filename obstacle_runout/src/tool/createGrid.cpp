#include "read_map/createGrid.h"
#include "ros/ros.h" //11.9ljl 加入ros回调函数
/*
**检测障碍物中每一个栅格0.7×0.7m
**PGM图片保存每一个像素pixel为0.05×0.05m
**所以14×14=196个像素块组成一个栅格
*/

IMG *newMAP;
YAML *newYAML;
double init_x,init_y;

void callback_final_data(const std_msgs::String::ConstPtr &msg)
{
    double real_time_x = 0,real_time_y = 0,real_time_heading = 0;
	string tmp = msg->data;
	stringstream stringin(tmp);
	stringin>>setprecision(10)>>real_time_x>>real_time_y>>real_time_heading;//heading 为弧度

	init_x = real_time_x;
	init_y = real_time_y;
	
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_to_final_data");
  ros::NodeHandle nh;
  ros::Subscriber final_data_sub;
  final_data_sub = nh.subscribe("/final_data", 1,callback_final_data);
  ros::spin();
  return 0;
}

/*
    void read_pgm_and_yaml()
    根据当前所在子图的编号读取相应的PGM和YAML文件
*/

void createGrid::read_pgm_and_yaml()
{

    cout<<"map_id_num: "<<map_id_num <<endl;
    string r = boost::lexical_cast<string>(map_id_num);
    string pgm = pgm_path + r + ".pgm";
    string yaml = yaml_path + r + ".yaml";
    newMAP = readmap.img_open(pgm);
    newYAML = readyaml.yaml_open(yaml);
    row = newMAP->rows;
    col = newMAP->cols;
    pix = newYAML->pixel;
    grid_row = 50; //11.9ljl 将生成栅格图改大    //ceil(row*pix/m_per_cell_);
    grid_col = 50; //11.9   //ceil(col*pix/m_per_cell_);
    pix_per_grid_num = floor(m_per_cell_/pix+0.5);
    grid_num = grid_row*grid_col-1;
    cout<<"read_pgm_and_yaml: "<<row<<" "<<col<<" "<<grid_row<<" "<<grid_col<<" "<<pix_per_grid_num<<" "<<grid_num<<endl;
}

/*
    GRID LocalToGlobal(GRID grid_tmp)
    将栅格在子图中的局部坐标转换为全局坐标
*/
GRID createGrid::LocalToGlobal(GRID grid_tmp)
{
    double last_kill_x = 0;
    double last_kill_y = 0;
    double last_kill_heading = 0;

    last_kill_x = start_point[map_id_num-1].x;
    last_kill_y = start_point[map_id_num-1].y;
    last_kill_heading = start_point[map_id_num-1].heading;

    is_map_change = -1;

	double send_x = grid_tmp.grid_x*cos(-last_kill_heading) - grid_tmp.grid_y*sin(-last_kill_heading)+last_kill_x;
	double send_y = grid_tmp.grid_y*cos(-last_kill_heading) + grid_tmp.grid_x*sin(-last_kill_heading)+last_kill_y;

	grid_tmp.grid_x = send_x;
	grid_tmp.grid_y = send_y;

    return grid_tmp;
}

/*
    vector<GRID> PGMToOriginMap(vector<GRID> grid_map,YAML *newYAML,int grid_num)
    更改PGM图坐标系的原点位置，原点从左下角变为机器人采图起始点
*/
vector<GRID> createGrid::PGMToOriginMap(vector<GRID> grid_map,YAML *newYAML,int grid_num)
{
    //ros::NodeHandle n;//这个NodeHandle写成全局
    vector<GRID> grid_map_tmp;
    GRID grid_tmp;
    //double init_x = newYAML->init_x;//改成回调函数finaldata x,y,theta ljl11.8
    //double init_y = newYAML->init_y;//
    for(int i=0; i<grid_num; i++){
        grid_tmp.grid_x = -grid_map.at(i).grid_x + init_x;
        grid_tmp.grid_y = -grid_map.at(i).grid_y + init_y;
        grid_tmp.occupied = grid_map.at(i).occupied;
        grid_tmp.i = grid_map.at(i).i;
        grid_tmp.j = grid_map.at(i).j;
        grid_map_tmp.push_back(grid_tmp);
    }
    return grid_map_tmp;
}

/*
    vector<GRID> OriginMapToGlobalMap(vector<GRID> grid_map,int grid_num)
    将每一个栅格中心点坐标从PGM坐标系转换为机器人所用的右手坐标系（在PGM图上看是向上为x轴正方向，向右为y轴正方向）
    1.有时PGM图图歪斜较多可设定theta值调整
    2.x轴与y轴交换
    3.坐标值做关于中心水平线（经x值的中心点做关于y轴的平行线）的翻转
    4.最后做子图坐标系转全局坐标系的转换
*/
vector<GRID> createGrid::OriginMapToGlobalMap(vector<GRID> grid_map,int grid_num)
{
    vector<GRID> grid_map_tmp;
    double tmp_x,tmp_y;
    double center_x = grid_map.at(grid_col/2*grid_row).grid_y;  //0625
    for(int i=0; i<grid_num; i++){
        GRID grid_tmp;
        tmp_x = grid_map.at(i).grid_x*cos(theta)-grid_map.at(i).grid_y*sin(theta);//1.
        tmp_y = grid_map.at(i).grid_x*sin(theta)+grid_map.at(i).grid_y*cos(theta);
        grid_tmp.grid_x = tmp_y;//2.
        grid_tmp.grid_y = tmp_x;
        grid_tmp.grid_x = 2*center_x - grid_tmp.grid_x; //3.
        grid_tmp.occupied = grid_map.at(i).occupied;
        grid_tmp.i = grid_map.at(i).i;
        grid_tmp.j = grid_map.at(i).j;

        grid_tmp = LocalToGlobal(grid_tmp);  //4.
        grid_map_tmp.push_back(grid_tmp);
    }
    return grid_map_tmp;
}

/*
    vector<GRID> createStaticGrid()
    创建静态栅格地图
*/
vector<GRID> createGrid::createStaticGrid()
{
    vector<GRID> grid_map;
    vector<GRID> static_grid_map_tmp;

    //grid_map初始化
    for(int i=0; i<grid_num; i++){
        GRID grid_map_tmp;
        grid_map_tmp.occupied = 0;
        grid_map_tmp.grid_x = grid_map_tmp.grid_y = 0;
        grid_map_tmp.i = grid_map_tmp.j = 0;
        grid_map.push_back(grid_map_tmp);
    }

    int grid_i_tmp = -1;
    int grid_j_tmp = -1;
    
    printf("grid_row,grid_col,pix_per_grid_num: %d %d %d\n",grid_row,grid_col,pix_per_grid_num);
    
    for(int i=0; i<col; i++)
    {
        for(int j=0; j<row; j++)
        {
            int grid_i = floor(i/pix_per_grid_num); //每一个像素点对应的栅格
            int grid_j = floor(j/pix_per_grid_num);
        //     if(newMAP->data[i*row+j]<=gray_threshold){ //白255,黑0
        //         grid_map[grid_i*grid_row+grid_j].occupied++;
        //     }
         }   //不需要修改occupied ljl 11.8
    }
    for(int i=0; i<grid_col; i++)
    {
        for(int j=0; j<grid_row; j++)
        {
            grid_map[i*grid_row+j].i = i;
            grid_map[i*grid_row+j].j = j;
            grid_map[i*grid_row+j].grid_x = -pix_per_grid_num*pix*(j+1/2);//计算每一个栅格中心坐标
            grid_map[i*grid_row+j].grid_y = -pix_per_grid_num*pix*(i+1/2);
        }
    }

    static_grid_map_tmp = PGMToOriginMap(grid_map,newYAML,grid_num);
    static_grid_map_tmp = OriginMapToGlobalMap(static_grid_map_tmp,grid_num);

    return static_grid_map_tmp;
}

//11.9添加成员函数vector<GRID> createGrid::PGMToOriginMap(vector<GRID> grid_map,double x ,dboule y,int grid_num)
//删去

//
//11.9回调函数
//
// void createGrid::callback_final_data(const std_msgs::String::ConstPtr &msg){
//     double real_time_x = 0,real_time_y = 0,real_time_heading = 0;
// 	string tmp = msg->data;
// 	stringstream stringin(tmp);
// 	stringin>>setprecision(10)>>real_time_x>>real_time_y>>real_time_heading;//heading 为弧度

// 	init_x = real_time_x;
// 	init_y = real_time_y;
	
// }
