#include <iostream>
#include <vector>
#include <queue>
#include <limits>
using namespace std;
/*************************************************
Author: wangpengfei
Date:2018-02-05
Description: a star algorithm
**************************************************/
#define DEBUGASTAR
const int INT_MAX = std::numeric_limits<int>::max();
/**
 * @brief  save map nodes index and F value
 * @note
 */
struct Index
{
    Index(int node_index,int node_valuef):index(node_index),valuef(node_valuef)
    {}
    int index;
    int valuef;
};
class greater_class{
public:
    bool operator()(const Index& compare_first, const Index& compare_second)//const
    {
        return compare_first.valuef > compare_second.valuef;
    }
};
/**
 * @brief a star algorithm
 * @note a star base process，sacrifice G value determination and close list detection , but enhance ability
 */
class Astar
{
public:
    Astar(vector<vector<int>>& map, int mapx, int mapy):map_(map), mapHeight_(mapx), mapWidth_(mapy)
    {
        mapSize_ = mapWidth_ * mapHeight_;
        valueG_ = new int[mapWidth_ * mapHeight_];
    }
    /**
   　 * @brief reset some param
   　 */
    void initialize();
    /**
     * @brief　a*　compute path
     * @param　start node
     * @param　end node
   　 */
    bool computePath(int start_x, int start_y, int end_x, int end_y);
    /**
     * @brief print path　end－>start
   　 */
    void printPath()const;
    ~Astar()
    {
        delete valueG_;
    };
private:
    inline int getMapIndex(int index_x, int index_y)const
    {
        return index_x*mapWidth_ + index_y;
    }
    /**
     * @brief  add surrounding nodes to open list
     * @param valueG_current  G value in ori_index node
     * @param　ori_index  center nodes index
     * @param　index waiting to be added to open list
     * @param　end node
   　 */
    void addSurroundingPoint(int valueG_current, int ori_index, int index);
private:
    vector<Index>queue_;
    int* valueG_;
    vector<vector<int> >map_;
    int mapWidth_;
    int mapHeight_;
    int mapSize_;
    int start_x_;
    int start_y_;
    int end_x_;
    int end_y_;
    int start_index_;
    int end_index_;
};
void Astar::initialize()
{
    //map_ = map;
    //mapWidth_ = mapy;
    //mapHeight_ = mapx;
    //mapSize_ = mapWidth_ * mapHeight_;
    //valueG_ = new int[mapWidth_ * mapHeight_];//????
}
bool Astar::computePath(int start_x, int start_y, int end_x, int end_y)
{
    //save start and end nodes
    start_x_ = start_x;
    start_y_ = start_y;
    end_x_ = end_x;
    end_y_ = end_y;
    //beyond map
    int start_index = getMapIndex(start_x, start_y);
    int end_index = getMapIndex(end_x, end_y);
    start_index_ = start_index;
    end_index_ = end_index;
    if((start_index<0)||(start_index>mapSize_))
    {
        cout<<"Error: start point beyond map."<<endl;
        return false;
    }
    if((end_index<0)||(end_index>mapSize_))
    {
        cout<<"Error: end point beyond map."<<endl;
        return false;
    }
    // obstacel detection
    if(map_[start_x][start_y]==1)
    {
        cout<<"Error: start point in obstacle."<<endl;
        return false;
    }
    if(map_[end_x][end_y]==1)
    {
        cout<<"Error: start point in obstacle."<<endl;
        return false;
    }
     //start a*
    queue_.push_back(Index(start_index,0));
    std::fill(valueG_,valueG_+mapSize_,INT_MAX);
    valueG_[start_index]=0;
    while(queue_.size()>0)
    {
        Index top = queue_[0];
#ifdef DEBUGASTAR
        cout<<"queue 111_:"<<endl;
        for(int i=0;i<queue_.size();i++)
        {
            cout<<queue_[i].index<<"("<<queue_[i].valuef<<") ";
        }
        cout<<endl;
        //cout<<"top index = "<<top.index<<" F="<<top.valuef <<" G = "<<valueG_[top.index]<<endl;
#endif
        pop_heap(queue_.begin(), queue_.end(), greater_class());//
#ifdef DEBUGASTAR
        cout<<"queue 222_:"<<endl;
        for(int i=0;i<queue_.size();i++)
        {
            cout<<queue_[i].index<<"("<<queue_[i].valuef<<") ";
        }
        cout<<endl;
        //cout<<"top index = "<<top.index<<" F="<<top.valuef <<" G = "<<valueG_[top.index]<<endl;
#endif        
        queue_.pop_back();
        if(end_index==top.index)
        {
            cout<<"we find end index "<<top.index<<endl;
            return true;
        }
        // left right top down
        addSurroundingPoint(valueG_[top.index], top.index, top.index-1);
        addSurroundingPoint(valueG_[top.index], top.index, top.index+1);
        addSurroundingPoint(valueG_[top.index], top.index, top.index-mapWidth_);
        addSurroundingPoint(valueG_[top.index], top.index, top.index+mapWidth_);
    }
    return false;
}
void Astar::addSurroundingPoint(int valueG_current, int ori_index, int index)
{
    if(index<0 || index>mapSize_)
        return;
    int index_x = index/mapWidth_;
    int index_y = index%mapWidth_;
    int ori_index_y = ori_index%mapWidth_;
    if((ori_index_y==mapWidth_-1) && (index_y==0))
        return;
    if((ori_index_y==0) && (index_y==mapWidth_-1))
        return;
    if(map_[index_x][index_y]==1)
        return;
    if(valueG_[index]<INT_MAX)
        return;
    int temp_G = valueG_current + 1;
    //manhadun distance
    int temp_H = abs(index_x - end_x_) + abs(index_y - end_y_);
    int temp_F = temp_H + temp_G;
    valueG_[index] = temp_G;
#ifdef DEBUGASTAR
    //cout<<"valueGcurrent="<<valueG_current<<endl;
    //cout<<"GF"<<temp_G<<" "<<temp_F<<endl;
#endif
    queue_.push_back(Index(index, temp_F));
    push_heap(queue_.begin(), queue_.end(), greater_class());
}
void Astar::printPath()const
{
//get path by valueG_
    vector<pair<int, int> > path;
    pair<int, int>current;
    current.first = end_x_;
    current.second = end_y_;
    path.push_back(current);
#ifdef DEBUGASTAR
    cout<<"valueG_"<<endl;
    for(int i=0;i<mapSize_;i++)
        cout<<valueG_[i]<<endl;
#endif
    while(getMapIndex(current.first,current.second)!=start_index_)
    {
        //research eight directon for G value
        int minval = INT_MAX;
        int minx = -1;
        int miny = -1;
        for(int i = -1; i <= 1; i++)
        {
          for(int j = -1; j <= 1; j++)
          {
            if(i==0 && j==0) continue;
        int tempx = current.first + i;
        int tempy = current.second + j;
        if(tempx<0 || tempx>mapHeight_-1 || tempy<0 || tempy>mapWidth_-1)
        continue;
            int tempG_index = getMapIndex(tempx, tempy);
            if(valueG_[tempG_index] < minval)
            {
                minval = valueG_[tempG_index];
                minx = current.first+i;
                miny = current.second+j;
            }
          }
        }
#ifdef DEBUGASTAR
    cout<<"minx = "<<minx<<endl;
    cout<<"miny = "<<miny<<endl;
#endif
        if(minx == -1 && miny == -1)
        {
            cout<<"can not find minimum neighbor."<<endl;
            return ;
        }
        current.first = minx;
        current.second = miny;
        path.push_back(current);
    }
    //print
    cout<<"astar path:"<<endl;
    for(auto iter = path.cbegin(); iter != path.cend(); iter++)
    {
        cout<<"("<<(*iter).first<<","<<(*iter).second<<")"<<endl;
    }
}
int main(int argc, char**argv)
{
    //define map
    //map explaination　０not obstacle 　１ obstacle
    //set nodes distance １　
    /** map
     * 0 1 0 1 1 1 0
     * 0 0 1 0 0 1 0(start)
     * 0 1 1 0 0 0 0
     * 0 0 0 0 0 1 1
     * 1 0(end) 0 1 0 0 0
   　 */
//vector initialize
    vector<vector<int> >map = {{0,1,0,1,1,1,0},{0,0,1,0,0,1,0},{0,1,1,0,0,0,0},{0,0,0,0,0,1,1},{1,0,0,1,0,0,0}};
    int mapx = 5;
    int mapy = 7;
    Astar astar(map, mapx, mapy);
    //load map
    //astar.initialize(map, mapx, mapy);
    //get path
    int startx = 1; //start from 0, marked in map
    int starty = 6;
    int endx = 4;
    int endy = 1;
    if(astar.computePath(startx, starty, endx, endy))
        astar.printPath();
    return 0;
}
