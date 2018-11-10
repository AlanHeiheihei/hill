#include <iostream>
#include <assert.h>
#include "dstar/B_Spline.h"

using namespace std;



void calc_b_spline_coefficient(int max_control_point_cnt_idx,int p,int max_knots_idx,std::vector<float> knots,
                                                                            float u,std::vector<float>& coeffis)
{
    //std::cout<<"max_knots_idx="<<max_knots_idx<<",max_control_point_cnt_idx="<<max_control_point_cnt_idx<<",p="<<p<<std::endl;
    assert(max_knots_idx==max_control_point_cnt_idx+p+1 && "m=n+1+p");

    for(int i=0;i<=max_control_point_cnt_idx;i++){ //初始化
        coeffis.push_back(0);
    }

    int m=max_knots_idx+1;
    float um=knots[m-1];
    float u0=knots[0];

    if(u==u0){
        coeffis[0]=1.0; //Array coeffis[ ] holds all intermediate and the final results. 
        return;
    }
    if(u==um){
        coeffis[max_control_point_cnt_idx]=1.0;
        return;
    }

    assert(u>=u0&&u<=um && "u is invalid!");

    //find u in which segment
    int k=-1;

    for(int i=0;i<m-1;i++){
        if(u>=knots[i] && u<knots[i+1]){
            k=i;
            break;
        }
    }

    coeffis[k]=1.0;

    for(int d=1;d<=p;d++)
    {
        if(k-d>=0)
        { // 从Nk-d+1,d-1(u)推出North-East方向的Nk-d,d(u)
            coeffis[k-d]=(knots[k+1]-u)/(knots[k+1]-knots[k-d+1])*coeffis[k-d+1];// right (south-west corner) term only
        }
        for(int i=k-d+1;i<=k-1;i++)
        { //compute internal terms 
            coeffis[i]=(u-knots[i])/(knots[i+d]-knots[i])*coeffis[i]+(knots[i+d+1]-u)/(knots[i+d+1]-knots[i+1])*coeffis[i+1];
        }
        coeffis[k]=(u-knots[k])/(knots[k+d]-knots[k])*coeffis[k];// 往south-east方向递推
    }
}