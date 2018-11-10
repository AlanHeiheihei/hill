
#ifndef DSTAR_H
#define DSTAR_H

#include <sstream>
#include <math.h>
#include <stack>
#include <queue>//STL
#include <list>//STL
#include <ext/hash_map>
#include <fstream>
using namespace std;
using namespace __gnu_cxx;

#define PI 3.1415926
#define DENSITY 10
#define TOP_LEFT 6
#define TOP_RIGHT 4
#define DOWN_LEFT 8
#define DOWN_RIGHT 2
class state {
 public:
  int x;
  int y;
  int orientation;//给出该点在某一时刻，某中心点的周围8个点中所处的位置，1-8（从中心点右手边顺时针编号）
  pair<double,double> k; //k1 & k2
  
  bool operator == (const state &s2) const {
    return ((x == s2.x) && (y == s2.y));
  }
  
  bool operator != (const state &s2) const {
    return ((x != s2.x) || (y != s2.y));
  }
  
  bool operator > (const state &s2) const {
    if (k.first-0.00001 > s2.k.first) return true;
    else if (k.first < s2.k.first-0.00001) return false;
    return k.second > s2.k.second;
  }

  bool operator <= (const state &s2) const {
    if (k.first < s2.k.first) return true;
    else if (k.first > s2.k.first) return false;
    return k.second < s2.k.second + 0.00001;
  }
  

  bool operator < (const state &s2) const {
    if (k.first + 0.000001 < s2.k.first) return true;
    else if (k.first - 0.000001 > s2.k.first) return false;
    return k.second < s2.k.second;
  }
   
};

struct ipoint2 {
  int x,y;
  double real_x,real_y;
  int cost;
};

typedef struct path {
  double x,y,heading;
}PATH;

struct cellInfo {

  double g;
  double rhs;
  double cost;

};

class state_hash {  //哈希函数，用来计算数据存储位置的hash值。struct也可以
 public:
  size_t operator()(const state &s) const {
    return s.x + 34245*s.y;
  }
};


typedef priority_queue<state, vector<state>, greater<state> > QUEUE_SMALL;//优先队列，小顶堆
typedef hash_map<state,cellInfo, state_hash, equal_to<state> > HASH_CELL;//state为key值，cellInfo为要存储的数据类型
typedef hash_map<state, float, state_hash, equal_to<state> > HASH_FLOAT;

typedef list<state> LIST_STATE;
typedef list< pair<ipoint2,double> > LIST_PAIR;


class Dstar {
  
 public:
    
  Dstar();
  void   init(int sX, int sY, int gX, int gY);
  void   updateCell(int x, int y, double val);
  void   updateStart(int x, int y);
  void   updateGoal(int x, int y);
  bool   replan();
  void   draw();
  void   drawCell(state s,float z);

  LIST_STATE getPath();
  
 private:
  
  LIST_STATE path;

  double C1;
  double k_m;
  state s_start, s_goal, s_last;
  int maxSteps;  

  QUEUE_SMALL openList;
  HASH_CELL cellHash;
  HASH_FLOAT openHash;

  bool   close(double x, double y);
  void   makeNewCell(state u);
  double getG(state u);
  double getRHS(state u);
  void   setG(state u, double g); //g(u)
  double setRHS(state u, double rhs); //rhs(u)
  double eightCondist(state a, state b); //8个方向
  int    computeShortestPath(); //
  void   updateVertex(state u);
  void   insert(state u);
  void   remove(state u);
  double trueDist(state a, state b);
  double heuristic(state a, state b); //启发值 h(u)
  state  calculateKey(state u);
  void   getSucc(state u, list<state> &s);
  void   getPred(state u, list<state> &s);
  double cost(state a, state b); 
  bool   occupied(state u);
  bool   isValid(state u);
  float  keyHashCode(state u);
  bool   JudgeIsAdjoinObstactlePoints(state u);//decoded by zbl on jul 5
};

#endif
