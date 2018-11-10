/* Dstar.cpp
 * James Neufeld (neufeld@cs.ualberta.ca)
 */

#include "dstar/Dstar.h"
#include <stdio.h>
#include <iostream>
#ifdef USE_OPEN_GL
#ifdef MACOS
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

std::ofstream cost_add_g("/home/deepdriving/shenrk/xc_Project/catkin_car/src/obstacle_runout/output/cost_add_g.txt");
Dstar::Dstar() { 

  maxSteps = 80000;  // node expansions before we give up
  C1       = 1;      // cost of an unseen cell

}

/* float Dstar::keyHashCode(state u) 
 * -------------------------- 
 * Returns the key hash code for the state u, this is used to compare
 * a state that have been updated
 */
float Dstar::keyHashCode(state u) {

  return (float)(u.k.first + 1193*u.k.second);

}

/* bool Dstar::isValid(state u) 
 * --------------------------
 * Returns true if state u is on the open list or not by checking if
 * it is in the hash table.
 */
bool Dstar::isValid(state u) {
  
  HASH_FLOAT::iterator cur = openHash.find(u);
  if (cur == openHash.end()) {
    cost_add_g<<"isvalid: ("<<u.x<<","<<u.y<<")"<<endl;
    return false;
  }
  if (!close(keyHashCode(u), cur->second)) return false;
  return true;
  
}

/* void Dstar::getPath() 
 * --------------------------
 * Returns the path created by replan()
 */
list<state> Dstar::getPath() {
  return path;
}

/* bool Dstar::occupied(state u)
 * --------------------------
 * returns true if the cell is occupied (non-traversable), false
 * otherwise. non-traversable are marked with a cost < 0.
 */
bool Dstar::occupied(state u) {
  
  HASH_CELL::iterator cur = cellHash.find(u);
  
  if (cur == cellHash.end()) return false;
  return (cur->second.cost < 0);
}

/* void Dstar::init(int sX, int sY, int gX, int gY)
 * --------------------------
 * Init dstar with start and goal coordinates, rest is as per
 * [S. Koenig, 2002]
 */
void Dstar::init(int sX, int sY, int gX, int gY) {
  
  cellHash.clear();
  path.clear();
  openHash.clear();
  while(!openList.empty()) 
    openList.pop(); //弹出当前最小值

  k_m = 0;
  
  s_start.x = sX;
  s_start.y = sY;
  s_goal.x  = gX;
  s_goal.y  = gY;

  cellInfo tmp;
  tmp.g = INFINITY;//zbl 
  tmp.rhs =  0;
  tmp.cost = C1;

  cellHash[s_goal] = tmp;
  insert(s_goal);//zbl
  tmp.g = tmp.rhs = INFINITY;//heuristic(s_start,s_goal); 
  tmp.cost = C1;
  cellHash[s_start] = tmp;
  s_start = calculateKey(s_start);

  s_last = s_start;

}

/* void Dstar::makeNewCell(state u)
 * --------------------------
 * Checks if a cell is in the hash table, if not it adds it in.
 */
void Dstar::makeNewCell(state u) {
  
  if (cellHash.find(u) != cellHash.end()) return;

  cellInfo tmp;
  tmp.g       = tmp.rhs = INFINITY;//heuristic(u,s_goal);
  tmp.cost    = C1;
  cellHash[u] = tmp;
  
}

/* double Dstar::getG(state u)
 * --------------------------
 * Returns the G value for state u.
 */
double Dstar::getG(state u) {

  if (cellHash.find(u) == cellHash.end()) 
    return heuristic(u,s_goal);
  return cellHash[u].g;
  
}

/* double Dstar::getRHS(state u)
 * --------------------------
 * Returns the rhs value for state u.
 */
double Dstar::getRHS(state u) {

  if (u == s_goal) return 0;  

  if (cellHash.find(u) == cellHash.end()) 
    return heuristic(u,s_goal);
  return cellHash[u].rhs;
  
}

/* void Dstar::setG(state u, double g)
 * --------------------------
 * Sets the G value for state u
 */
void Dstar::setG(state u, double g) {
  
  makeNewCell(u);  
  cellHash[u].g = g; 
}

/* void Dstar::setRHS(state u, double rhs)
 * --------------------------
 * Sets the rhs value for state u
 */
double Dstar::setRHS(state u, double rhs) {
  
  makeNewCell(u);
  cellHash[u].rhs = rhs;

}

/* double Dstar::eightCondist(state a, state b) 
 * --------------------------
 * Returns the 8-way distance between state a and state b.
 */
double Dstar::eightCondist(state a, state b) {
  double temp;
  double min = abs(a.x - b.x);
  double max = abs(a.y - b.y);
  if (min > max) {
    double temp = min;
    min = max;
    max = temp;
  }
  return ((M_SQRT2-1.0)*min + max); /* M_SQRT2 == sqrt(2) */
}

/* int Dstar::computeShortestPath()
 * --------------------------
 * As per [S. Koenig, 2002] except for 2 main modifications:
 * 1. We stop planning after a number of steps, 'maxsteps' we do this
 *    because this algorithm can plan forever if the start is
 *    surrounded by obstacles. 
 * 2. We lazily remove states from the open list so we never have to
 *    iterate through it.
 */
int Dstar::computeShortestPath() {
  
  list<state> s;
  list<state>::iterator i;

  if (openList.empty()) return 1;

  cout<<"openList.top(): "<<openList.top().x<<" "<<openList.top().y<<" "<<openList.top().k.first<<" "<<openList.top().k.second<<endl;

  int k=0;
  while ((!openList.empty()) && 
         (openList.top() < (s_start = calculateKey(s_start))) || 
         (getRHS(s_start) != getG(s_start))) {
    
    cost_add_g<<"bool: "<<openList.empty()<<" "<<(openList.top() < (s_start = calculateKey(s_start)))<<" "<<(getRHS(s_start) != getG(s_start))<<endl;
        if (k++ > maxSteps) {
      fprintf(stderr, "At maxsteps\n");
      return -1;
    }


    state u;

    bool test = (getRHS(s_start) != getG(s_start));
    
    // lazy remove
    while(1) { 
      if (openList.empty()) return 1;
      u = openList.top();
      openList.pop();
      
      if (!isValid(u)) continue;
      if (!(u < s_start) && (!test)) return 2;
      break;
    }
    
    HASH_FLOAT::iterator cur = openHash.find(u);
    openHash.erase(cur);

    state k_old = u;

    if(u.x==56 && u.y == 40){
       cost_add_g<<"222set : ("<<u.x<<","<<u.y<<") gval: "<<getG(u)<<" rhs: "<<getRHS(u)<<endl;
    }

    if (k_old < calculateKey(u)) { // u is out of date
      insert(u);
    } else if (getG(u) > getRHS(u)) { // needs update (got better)
      setG(u,getRHS(u));
      cost_add_g<<"set : ("<<u.x<<","<<u.y<<") gval: "<<getG(u)<<endl;
      getPred(u,s);
      for (i=s.begin();i != s.end(); i++) {
        updateVertex(*i);
      }
    } else {   // g <= rhs, state has got worse
      cost_add_g<<"inf: ("<<u.x<<","<<u.y<<")" <<"g: "<<getG(u)<<endl;
      setG(u,INFINITY);
      getPred(u,s);
      for (i=s.begin();i != s.end(); i++) {
        updateVertex(*i);
      }
      updateVertex(u);
    }
  }
  return 0;
}

/* bool Dstar::close(double x, double y) 
 * --------------------------
 * Returns true if x and y are within 10E-5, false otherwise
 */
bool Dstar::close(double x, double y) {
    
  if (isinf(x) && isinf(y)) return true;
  return (fabs(x-y) < 0.00001);
  
}

/* void Dstar::updateVertex(state u)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
void Dstar::updateVertex(state u) {

  list<state> s;
  list<state>::iterator i;
 
  if (u != s_goal) {
    getSucc(u,s);//找出周围8个栅格，放在s中
    double tmp = INFINITY;
    double tmp2;

    for (i=s.begin();i != s.end(); i++) {
      tmp2 = getG(*i) + cost(u,*i);//rhs(i)
      if (tmp2 < tmp) tmp = tmp2; //8个中找一个rhs值最小的
    }
    if (!close(getRHS(u),tmp)) setRHS(u,tmp);
  }

  if (!close(getG(u),getRHS(u))) {
    cost_add_g<<"update: ("<<u.x<<","<<u.y<<")  rhs: "<<getRHS(u)<<endl;
    insert(u); //u为目标点
  }
}

/* void Dstar::insert(state u) 
 * --------------------------
 * Inserts state u into openList and openHash.
 */
void Dstar::insert(state u) {
  
  HASH_FLOAT::iterator cur;
  float csum;

  u    = calculateKey(u);
  cur  = openHash.find(u);
  csum = keyHashCode(u);
  // return if cell is already in list. TODO: this should be
  // uncommented except it introduces a bug, I suspect that there is a
  // bug somewhere else and having duplicates in the openList queue
  // hides the problem...
  //if ((cur != openHash.end()) && (close(csum,cur->second))) return;
  
  openHash[u] = csum;
  openList.push(u);
} 

/* void Dstar::remove(state u)
 * --------------------------
 * Removes state u from openHash. The state is removed from the
 * openList lazilily (in replan) to save computation.
 */
void Dstar::remove(state u) {
  
  HASH_FLOAT::iterator cur = openHash.find(u);
  if (cur == openHash.end()) return;
  openHash.erase(cur);
}


/* double Dstar::trueDist(state a, state b) 
 * --------------------------
 * Euclidean cost between state a and state b.
 */
double Dstar::trueDist(state a, state b) {
  
  float x = a.x-b.x;
  float y = a.y-b.y;
  return sqrt(x*x + y*y);
  
}

/* double Dstar::heuristic(state a, state b)
 * --------------------------
 * Pretty self explanitory, the heristic we use is the 8-way distance
 * scaled by a constant C1 (should be set to <= min cost).
 */
double Dstar::heuristic(state a, state b) {
  return eightCondist(a,b)*C1;
}

/* state Dstar::calculateKey(state u)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
state Dstar::calculateKey(state u) {
  
  double g=getG(u);
  double rhs=getRHS(u);
  //double val = fmin(g,rhs);
  if (g > rhs) {
  
  u.k.first  = rhs + 4*heuristic(u,s_start) + k_m; //为什么×4？
  u.k.second = rhs;}

  else {
  
  u.k.first  = g + heuristic(u,s_start) + k_m;
  u.k.second = g;
  }
  /*if(u.x == 25 && u.y == 36 || u.x == 25 && u.y == 35){
    u.k.first  = 55;
  }*/
  cost_add_g<<"kkkkk: ("<<u.x<<","<<u.y<<")  u.k.first : "<<u.k.first<<"  u.k.second : "<<u.k.second<<endl;
  return u;

}

/* double Dstar::cost(state a, state b)
 * --------------------------
 * Returns the cost of moving from state a to state b. This could be
 * either the cost of moving off state a or onto state b, we went with
 * the former. This is also the 8-way cost.
 */
std::ofstream ob_neighbour("/home/deepdriving/shenrk/xc_Project/catkin_car/src/obstacle_runout/output/top.txt");
double Dstar::cost(state a, state b) {

  int xd = abs(a.x-b.x);
  int yd = abs(a.y-b.y);
  double scale = 1;
  int MAGCost = 5;

  if (xd+yd>1) scale = M_SQRT2; //斜边sacle值为sqrt(2)，直行为1
  if (cellHash.count(a) == 0) return scale*C1;//count：查找在hash表中，关键字a的个数
  if (xd+yd>1){
    if(JudgeIsAdjoinObstactlePoints(b)){//判断点周围是否与中心点临接相同的障碍物点
    //cout<<"00000"<<endl;
    ob_neighbour<<"center:("<<a.x<<","<<a.y<<")  "<<"neighbour:("<<b.x<<","<<b.y<<")"<<endl;
    return scale*cellHash[a].cost*MAGCost;
    }
    else return scale*cellHash[a].cost;
  }
  return scale*cellHash[a].cost;

}
/* void Dstar::updateCell(int x, int y, double val)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
void Dstar::updateCell(int x, int y, double val) {
  
   state u;
  
  u.x = x;
  u.y = y;

  if ((u == s_start) || (u == s_goal)) return;

  makeNewCell(u); 
  cellHash[u].cost = val;

  //updateVertex(u); 
}

/* void Dstar::getSucc(state u,list<state> &s)
 * --------------------------
 * Returns a list of successor states for state u, since this is an
 * 8-way graph this list contains all of a cells neighbours. Unless
 * the cell is occupied in which case it has no successors. 
 */
void Dstar::getSucc(state u,list<state> &s) {
  
  s.clear();
  u.k.first  = -1;
  u.k.second = -1;

  if (occupied(u)) return;

  u.x += 1;
  u.orientation = 1;//记录方位
  s.push_front(u);
  u.y += 1;
  u.orientation = 2;//记录方位
  s.push_front(u);
  u.x -= 1;
  u.orientation = 3;//记录方位
  s.push_front(u);
  u.x -= 1;
  u.orientation = 4;//记录方位
  s.push_front(u);
  u.y -= 1;
  u.orientation = 5;//记录方位
  s.push_front(u);
  u.y -= 1;
  u.orientation = 6;//记录方位
  s.push_front(u);
  u.x += 1;
  u.orientation = 7;//记录方位
  s.push_front(u);
  u.x += 1;
  u.orientation = 8;//记录方位
  s.push_front(u);//按右手坐标系，左上右下的顺序找出当前u栅格的周围8个栅格

}

/* void Dstar::getPred(state u,list<state> &s)
 * --------------------------
 * Returns a list of all the predecessor states for state u. Since
 * this is for an 8-way connected graph the list contails all the
 * neighbours for state u. Occupied neighbours are not added to the
 * list.
 */
void Dstar::getPred(state u,list<state> &s) {
  
  s.clear();
  u.k.first  = -1;
  u.k.second = -1;

  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  u.y += 1;
  if (!occupied(u)) s.push_front(u);
  u.x -= 1;
  if (!occupied(u)) s.push_front(u);
  u.x -= 1;
  if (!occupied(u)) s.push_front(u);
  u.y -= 1;
  if (!occupied(u)) s.push_front(u);
  u.y -= 1;
  if (!occupied(u)) s.push_front(u);
  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  
}

/* void Dstar::updateStart(int x, int y)
 * --------------------------
 * Update the position of the robot, this does not force a replan.
 */
void Dstar::updateStart(int x, int y) {

  s_start.x = x;
  s_start.y = y;
  
  k_m += heuristic(s_last,s_start);

  s_start = calculateKey(s_start);
  s_last  = s_start;
  
}

/* void Dstar::updateGoal(int x, int y)
 * --------------------------
 * This is somewhat of a hack, to change the position of the goal we
 * first save all of the non-empty on the map, clear the map, move the
 * goal, and re-add all of non-empty cells. Since most of these cells
 * are not between the start and goal this does not seem to hurt
 * performance too much. Also it free's up a good deal of memory we
 * likely no longer use.
 */
void Dstar::updateGoal(int x, int y) {
   
  LIST_PAIR toAdd;
  pair<ipoint2, double> tp;
  
  HASH_CELL::iterator i;
  LIST_PAIR::iterator kk;
  
  for(i=cellHash.begin(); i!=cellHash.end(); i++) {
    if (!close(i->second.cost, C1)) {
      tp.first.x = i->first.x;
      tp.first.y = i->first.y;
      tp.second = i->second.cost;
      toAdd.push_back(tp);
    }
  }

  cellHash.clear();
  openHash.clear();

  while(!openList.empty())
    openList.pop();
  
  k_m = 0;
  
  s_goal.x  = x;
  s_goal.y  = y;

  cellInfo tmp;
  tmp.g = tmp.rhs =  0;
  tmp.cost = C1;

  cellHash[s_goal] = tmp;

  tmp.g = tmp.rhs = heuristic(s_start,s_goal);
  tmp.cost = C1;
  cellHash[s_start] = tmp;
  s_start = calculateKey(s_start);

  s_last = s_start;    

  for (kk=toAdd.begin(); kk != toAdd.end(); kk++) {
    updateCell(kk->first.x, kk->first.y, kk->second);
  }
  

}

/* bool Dstar::replan()
 * --------------------------
 * Updates the costs for all cells and computes the shortest path to
 * goal. Returns true if a path is found, false otherwise. The path is
 * computed by doing a greedy search over the cost+g values in each
 * cells. In order to get around the problem of the robot taking a
 * path that is near a 45 degree angle to goal we break ties based on
 *  the metric euclidean(state, goal) + euclidean(state,start). 
 */

bool Dstar::replan() {

  path.clear();

  int res = computeShortestPath();
  std::cout<<"res: "<<res<<endl;
  //printf("res: %d ols: %d ohs: %d tk: [%f %f] sk: [%f %f] sgr: (%f,%f)\n",res,openList.size(),openHash.size(),openList.top().k.first,openList.top().k.second, s_start.k.first, s_start.k.second,getRHS(s_start),getG(s_start));
  if (res < 0) {
    fprintf(stderr, "NO PATH TO GOAL\n");
    return false;
  }
  list<state> n;
  list<state>::iterator i;

  state cur = s_start; 

  if (isinf(getG(s_start))) {
    fprintf(stderr, "NO PATH TO GOAL\n");
    return false;
  }
  
  while(cur != s_goal) {
    
    path.push_back(cur);
    getSucc(cur, n);

    if (n.empty()) {
      fprintf(stderr, "NO PATH TO GOAL\n");
      return false;
    }

    double cmin = INFINITY;
    double tmin;
    state smin;
    
    for (i=n.begin(); i!=n.end(); i++) {
  
      if (occupied(*i)) continue;
      double val  = cost(cur,*i);
      cost_add_g<<"("<<i->x<<","<<i->y<<")"<<"cost: "<<val;
      double val2 = trueDist(*i,s_goal) + trueDist(s_start,*i); // (Euclidean) cost to goal + cost to pred
      val += getG(*i);
      cost_add_g<<" val: "<<val<<endl;
      if (close(val,cmin)) {
        if (tmin > val2) {
          tmin = val2;
          cmin = val;
          smin = *i;
        }
      } else if (val < cmin) {
        tmin = val2;
        cmin = val;
        smin = *i;
      }
    }
    n.clear();
    cur = smin;
  }
  path.push_back(s_goal);
  return true;
}

bool Dstar::JudgeIsAdjoinObstactlePoints(state u){
  state point[2];
  switch(u.orientation){
    case TOP_LEFT:{         
          point[0] = u;
          point[1] = u;
          point[0].x++;
          point[1].y++;
          if(cellHash.count(point[0])==0){
             //b_neighbour<<"TOP_LEFT: "<<"("<<point[0].x<<","<<point[0].y<<") no val:"<<endl;
          }
          if(cellHash.count(point[1])==0){
             //ob_neighbour<<"TOP_LEFT: "<<"("<<point[1].x<<","<<point[1].y<<") no val:"<<endl;
          }
          //ob_neighbour<<"TOP_LEFT: "<<"("<<point[0].x<<","<<point[0].y<<") val:"<<cellHash[point[0]].cost<<" "<<"("<<point[1].x<<","<<point[1].y<<") val:"<<cellHash[point[1]].cost<<endl;
          break;
    }
    case TOP_RIGHT:{
          point[0] = u;
          point[1] = u;
          point[0].y--;
          point[1].x++;
          if(cellHash.count(point[0])==0){
             //ob_neighbour<<"TOP_RIGHT: "<<"("<<point[0].x<<","<<point[0].y<<") no val:"<<endl;
          }
          if(cellHash.count(point[1])==0){
             //ob_neighbour<<"TOP_RIGHT: "<<"("<<point[1].x<<","<<point[1].y<<") no val:"<<endl;
          }
          //ob_neighbour<<"TOP_RIGHT: "<<"("<<point[0].x<<","<<point[0].y<<") val:"<<cellHash[point[0]].cost<<" "<<"("<<point[1].x<<","<<point[1].y<<") val:"<<cellHash[point[1]].cost<<endl;
          break;
    }
    case DOWN_LEFT:{
          point[0] = u;
          point[1] = u;
          point[0].y++;
          point[1].x--;
          if(cellHash.count(point[0])==0){
             //ob_neighbour<<"DOWN_LEFT: "<<"("<<point[0].x<<","<<point[0].y<<") no val:"<<endl;
          }
          if(cellHash.count(point[1])==0){
             //ob_neighbour<<"DOWN_LEFT: "<<"("<<point[1].x<<","<<point[1].y<<") no val:"<<endl;
          }
          //ob_neighbour<<"DOWN_LEFT: "<<"("<<point[0].x<<","<<point[0].y<<") val:"<<cellHash[point[0]].cost<<" "<<"("<<point[1].x<<","<<point[1].y<<") val:"<<cellHash[point[1]].cost<<endl;
          break;
    }
    case DOWN_RIGHT:{
          point[0] = u;
          point[1] = u;
          point[0].y--;
          point[1].x--;
          if(cellHash.count(point[0])==0){
             //ob_neighbour<<"DOWN_RIGHT: "<<"("<<point[0].x<<","<<point[0].y<<") no val:"<<endl;
          }
          if(cellHash.count(point[1])==0){
             //ob_neighbour<<"DOWN_RIGHT: "<<"("<<point[1].x<<","<<point[1].y<<") no val:"<<endl;
          }
          //ob_neighbour<<"DOWN_RIGHT: "<<"("<<point[0].x<<","<<point[0].y<<") val:"<<cellHash[point[0]].cost<<" "<<"("<<point[1].x<<","<<point[1].y<<") val:"<<cellHash[point[1]].cost<<endl;
          break;
    }
    default:return false;
  }
  //cout<<"1234"<<endl;
  if(cellHash[point[0]].cost <0 ||cellHash[point[1]].cost <0){
    ob_neighbour<<"zqq cost 0&1: "<<cellHash[point[0]].cost<<" "<<cellHash[point[1]].cost<<endl;
    return true;
  }
  else return false;
}
#ifdef USE_OPEN_GL

void Dstar::draw() {

  HASH_CELL::iterator iter;
  HASH_FLOAT::iterator iter1;
  state t;

  list<state>::iterator iter2;
  
  glBegin(GL_QUADS);
  for(iter=cellHash.begin(); iter != cellHash.end(); iter++) {
    if (iter->second.cost == 1) glColor3f(0,1,0);
    else if (iter->second.cost < 0 ) glColor3f(1,0,0);
    else glColor3f(0,0,1);
    drawCell(iter->first,0.45);
  }

  glColor3f(1,1,0);
  drawCell(s_start,0.45);
  glColor3f(1,0,1);
  drawCell(s_goal,0.45);

  for(iter1=openHash.begin(); iter1 != openHash.end(); iter1++) {
    glColor3f(0.4,0,0.8);
    drawCell(iter1->first, 0.2);
  }

  
  glEnd();

  glLineWidth(4);
  glBegin(GL_LINE_STRIP);
  glColor3f(0.6, 0.1, 0.4);

  for(iter2=path.begin(); iter2 != path.end(); iter2++) {
    glVertex3f(iter2->x, iter2->y, 0.2);
  }
  glEnd();

}

void Dstar::drawCell(state s, float size) {

  float x = s.x;
  float y = s.y;
  
  
  glVertex2f(x - size, y - size);
  glVertex2f(x + size, y - size);
  glVertex2f(x + size, y + size);
  glVertex2f(x - size, y + size);


}

#else
void Dstar::draw() {}
void Dstar::drawCell(state s, float z) {}
#endif
