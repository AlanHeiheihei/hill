
#ifndef _B_SPLINE_H_
#define _B_SPLINE_H_

#include <vector>


void calc_b_spline_coefficient(int max_control_point_cnt_idx,int p,int max_knots_idx,std::vector<float> knots,
                                                                         float u,std::vector<float>& coeffis);


#endif