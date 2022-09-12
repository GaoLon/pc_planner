/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef VOXEL_MAP_HPP
#define VOXEL_MAP_HPP
#include "voxel_dilater.hpp"
#include "voxel_surf.hpp"
#include <memory>
#include <vector>
#include <Eigen/Eigen>
#define inf  1>>20 
using namespace std;
using namespace Eigen;
namespace voxel_map
{

    constexpr uint8_t Unoccupied = 0;
    constexpr uint8_t Occupied = 1;
    constexpr uint8_t ok = 1;
    constexpr uint8_t not_ok = 0;
    constexpr uint8_t Dilated = 2;
    
    struct GridNode;
    typedef GridNode* GridNodePtr;  /*    GridNodePtr：指向struct GridNode结构体类型的指针变量    */
    struct GridNode
    {     
        int id;        // 1--> open set, -1 --> closed set
        Eigen::Vector3d coord; 
        Eigen::Vector3i index;
        
        double gScore, fScore;
        GridNodePtr cameFrom;
        std::multimap<double, GridNodePtr>::iterator nodeMapIt;

        GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord){  
            id = 0;
            index = _index;
            coord = _coord;

            gScore = inf;
            fScore = inf;
            cameFrom = NULL;
        }

        GridNode(){};
        ~GridNode(){};
    };

    class VoxelMap
    {

    public:
        double all_size=0;
        VoxelMap() = default;
        VoxelMap(const Eigen::Vector3i &size, //xyz
                 const Eigen::Vector3d &origin, //offset
                 const double &voxScale)//resolution
            : mapSize(size),
              o(origin),
              scale(voxScale),
              voxNum(mapSize.prod()),
              step(1, mapSize(0), mapSize(1) * mapSize(0)),
              oc(o + Eigen::Vector3d::Constant(0.5 * scale)),
              bounds((mapSize.array() - 1) * step.array()),
              stepScale(step.cast<double>().cwiseInverse() * scale),//m1除m2   即m1.cwiseProduct(m2.cwiseInverse())   cwiseInverse是倒数
              voxels(voxNum, Unoccupied),
              voxel_begin(voxNum, Unoccupied),
              voxel_is_normal_ok_for_rrt(voxNum, not_ok),
              voxel_angle_height_ok(voxNum, not_ok),
              voxel_end_feasible(voxNum, not_ok),
              voxel_connect_information_without_dilate(voxNum),
              voxel_connect_information_on_end_feasible(voxNum),
              father(voxNum),
              rank(voxNum) {}
        typedef std::shared_ptr<VoxelMap> Ptr;

        GridNodePtr *** GridNodeMap;
    private:
        Eigen::Vector3i mapSize;
        Eigen::Vector3d o;
        double scale;
        int voxNum;std::vector<int> father;std::vector<int> rank;
        Eigen::Vector3i step;
        Eigen::Vector3d oc;
        Eigen::Vector3i bounds;
        Eigen::Vector3d stepScale;
        std::vector<uint8_t> voxels;//dialte之后occ和dialte的voxels都是occupied
        std::vector<uint8_t> voxel_begin;//放一开始的没有dialte的值,occ才occ,query用的这个
        std::vector<uint8_t> voxel_is_normal_ok_for_rrt;
        std::vector<uint8_t> voxel_angle_height_ok;//放一开始的没有dialte的值
        std::vector<uint8_t> voxel_end_feasible;//放最终的可行信息
        std::vector<std::vector<Eigen::Vector3i>> voxel_connect_information_without_dilate;
        std::vector<std::vector<Eigen::Vector3i>> voxel_connect_information_on_end_feasible;
        std::vector<Eigen::Vector3i> surf;
        std::vector<Eigen::Vector3i> end_feasible_surf;

    public:
        inline Eigen::Vector3i getSize(void) const
        {
            return mapSize;
        }
        inline Eigen::Vector3i getstep(void) const
        {
            return step;
        }

        inline double getScale(void) const
        {
            return scale;
        }

        inline Eigen::Vector3d getOrigin(void) const
        {
            return o;
        }

        inline Eigen::Vector3d getCorner(void) const
        {
            return mapSize.cast<double>() * scale + o;
        }

        inline const std::vector<uint8_t> &getVoxels(void) const
        {
            return voxels;
        }

        inline void setOccupied(const Eigen::Vector3d &pos)
        {

            const Eigen::Vector3i id = ((pos - o) / scale).cast<int>();
            
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
            
                all_size++;
                voxels[id.dot(step)] = Occupied;
                voxel_begin[id.dot(step)] = Occupied;
            }
        }

        inline void setOccupied(const Eigen::Vector3i &id)
        {
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                voxels[id.dot(step)] = Occupied;
                voxel_begin[id.dot(step)] = Occupied;
            }
        }
        inline void set_angle_height_ok(const Eigen::Vector3d &pos)
        {
            const Eigen::Vector3i id = ((pos - o) / scale).cast<int>();
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                voxel_angle_height_ok[id.dot(step)]  = ok;
                voxel_end_feasible[id.dot(step)]  = ok;
            }
        }
        inline void set_end_feasible_not_ok(const Eigen::Vector3i &id)
        {
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                voxel_end_feasible[id.dot(step)]  = not_ok;
            }
        }
        inline void set_normal_for_rrt_is_ok(const Eigen::Vector3d &pos)
        {
            const Eigen::Vector3i id = ((pos - o) / scale).cast<int>();
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                voxel_is_normal_ok_for_rrt[id.dot(step)]  = ok;
            }
        }


        inline void set_connect_information_without_dilate(const Eigen::Vector3d &pos_now,Eigen::Vector3i &index_neighbor)
        {
            const Eigen::Vector3i id = ((pos_now - o) / scale).cast<int>();
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                voxel_connect_information_without_dilate[id.dot(step)].push_back(index_neighbor);
            }
        }

        inline void set_connect_information_on_end_feasible(const Eigen::Vector3d &pos_now,Eigen::Vector3i &index_neighbor)
        {
            const Eigen::Vector3i id = ((pos_now - o) / scale).cast<int>();
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                voxel_connect_information_on_end_feasible[id.dot(step)].push_back(index_neighbor);
            }
        }
        inline int find_Father_until_root(int x){
            return x==father[x] ? x:(father[x] = find_Father_until_root(father[x]));
            //如果这个点的father是他自己，那就返回他自己，如果不是，则再往上回溯父节点，并且把沿途结点的父结点设为根结点。直到到达根结点
        }
        inline void merge(int i,int j){
            int i_root=find_Father_until_root(i);
            int j_root=find_Father_until_root(j);
            if(rank[i_root]<=rank[j_root])
                father[i_root]=j_root;
            else
                father[j_root]=i_root;
            if(rank[i_root]==rank[j_root] && i_root != j_root)//如果深度相同且根结点不同，则说明合并之后的根结点深度+1
                rank[j_root]++;//如果不同，则会把大的往小的那合并,大的那个往下走子数结点是不变的,大的那个也不变
            // father[i_root]=j_root;
        }
        inline void construct_disjoint_Set(){
            //init
            std::cout<<"mapSize(0)"<<mapSize(0)<<std::endl;
            std::cout<<"mapSize(1)"<<mapSize(1)<<std::endl;
            std::cout<<"mapSize(2)"<<mapSize(2)<<std::endl;
            std::cout<<"voxNum"<<voxNum<<std::endl;
            for(int i=0;i<voxNum;i++)
            {
                father[i]=i;
                rank[i]=1;//秩是以该节点作为根结点的子树的深度，一开始每个点的father都是他自己
            }
            //我麻了 我找到错误了....卡了3个点才发现是因为我的father当时复制粘贴直接用的uint8_t  这玩意是unsigned char 我把int的东西扔进去存直接废了
            //遍历map上所有的点，把有边和边连通信息的合并
            for(int i=0;i<mapSize(0);i++){
            for(int j=0;j<mapSize(1);j++){
            for(int k=0;k<mapSize(2);k++)
            {
                Eigen::Vector3i current_index(i,j,k);
                Eigen::Vector3d current_pos=posI2D(current_index);
                for(int num=0;num<voxel_connect_information_on_end_feasible[current_index.dot(step)].size();num++)
                {
                    Eigen::Vector3i connect_neighbor=voxel_connect_information_on_end_feasible[current_index.dot(step)][num];
                    Eigen::Vector3d neighbor_pos=posI2D(connect_neighbor);
                    merge(connect_neighbor.dot(step),current_index.dot(step));
                }
            }
            }
            }
        }
        inline bool query_if_angle_height_ok(const Eigen::Vector3d &pos) const
        {
            const Eigen::Vector3i id = ((pos - o) / scale).cast<int>();
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                return voxel_angle_height_ok[id.dot(step)];
            }
            else
            {   
                return false;
            }
        }

        inline bool query_if_normal_for_rrt_ok(const Eigen::Vector3d &pos) const
        {
            const Eigen::Vector3i id = ((pos - o) / scale).cast<int>();
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                return voxel_is_normal_ok_for_rrt[id.dot(step)];
            }
            else
            {   
                return false;
            }
        }

        inline bool query_if_angle_height_ok(const Eigen::Vector3i &id) const
        {
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                return voxel_angle_height_ok[id.dot(step)];
            }
            else
            {   
                return false;
            }
        }
        inline bool query_if_end_feasible_ok(const Eigen::Vector3i &id) const
        {
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                return voxel_end_feasible[id.dot(step)];
            }
            else
            {   
                return false;
            }
        }
        inline void dilate()
        {   

                std::vector<Eigen::Vector3i> lvec, cvec;
                lvec.reserve(voxNum);
                cvec.reserve(voxNum);
                int i, j, k, idx;
                bool check;
                for (int x = 0; x <= bounds(0); x++)
                {
                    for (int y = 0; y <= bounds(1); y += step(1))
                    {
                        for (int z = 0; z <= bounds(2); z += step(2))
                        {
                            if (voxels[x + y + z] == Occupied)
                            {
                                VOXEL_DILATER(i, j, k,
                                              x, y, z,
                                              step(1), step(2),
                                              bounds(0), bounds(1), bounds(2),
                                              check, voxels, idx, Dilated, cvec)
                            }
                        }
                    }
                }
            
                surf = cvec;
              

        }

        inline void caculate_end_Feasible_surf(int robot_height_min,int robot_height_max)
        {   
                
                //voxel_end_feasible是肯定满足robot的最低高度上没障碍物的，太保守
                //对于那些一直到最高高度也没有障碍物的需要check
                std::vector<uint8_t> voxel_for_sfc(voxNum,Unoccupied);
                for (int x = 0; x <= bounds(0); x++){
                    for (int y = 0; y <= bounds(1); y += step(1)){
                        for (int z = 0; z <= bounds(2); z += step(2))
                        {
                            if(voxel_end_feasible[x + y + z]== Occupied)
                            {
                                int dz=0;
                                // for(dz=0;dz<=robot_height_min*step(2);dz+=step(2))
                                //     voxel_for_sfc[x + y + z+dz]=Occupied;
                                for(dz=robot_height_min*step(2);dz<=robot_height_max*step(2);dz+=step(2))
                                { 
                                    if(voxel_begin[x + y + z+dz]==Unoccupied)//不仅如此，应该是遇到了一个occ就不再往上寻找了
                                        voxel_for_sfc[x + y + z+dz]=Occupied;    
                                    else//加了这个break 把这个问题解决之后那个蚊帐corridor z轴悬空联通又不出了。。
                                        break;//想想确实应该如此，除非没有根据边缘腐蚀出radius,不然旋转楼梯那里是不会连通的
                                }                              
                            }
                        }
                    }
                }

                std::vector<Eigen::Vector3i> lvec, cvec;
                lvec.reserve(voxNum);
                cvec.reserve(voxNum);
                int i, j, k, idx;
                bool check;
                for (int x = 0; x <= bounds(0); x++)
                {
                    for (int y = 0; y <= bounds(1); y += step(1))
                    {
                        for (int z = 0; z <= bounds(2); z += step(2))
                        {
                            if (voxel_for_sfc[x + y + z] == Occupied)
                            {
                                // //把最外面的一层作为蚊帐，而不是dialte之后（即又大了一圈的）的那层
                                // VOXEL_surf(i, j, k,
                                //             x, y, z,
                                //             step(1), step(2),
                                //             bounds(0), bounds(1), bounds(2),
                                //             check, voxel_for_sfc, idx, cvec)
                            
                                VOXEL_DILATER(i, j, k,
                                    x, y, z,
                                    step(1), step(2),
                                    bounds(0), bounds(1), bounds(2),
                                    check, voxel_for_sfc, idx, Dilated, cvec)
                            
                            }
                        }
                    }
                }
                end_feasible_surf= cvec;

        }

        inline void getSurfInBox(const Eigen::Vector3i &center,
                                 const int &halfWidth,
                                 std::vector<Eigen::Vector3d> &points) const
        {
            for (const Eigen::Vector3i &id : surf)
            {
                if (std::abs(id(0) - center(0)) <= halfWidth &&
                    std::abs(id(1) / step(1) - center(1)) <= halfWidth &&
                    std::abs(id(2) / step(2) - center(2)) <= halfWidth)
                {
                    points.push_back(id.cast<double>().cwiseProduct(stepScale) + oc);
                }
            }

            return;
        }

        inline void getSurf(std::vector<Eigen::Vector3d> &points) const
        {
            points.reserve(surf.size());
            for (const Eigen::Vector3i &id : surf)
            {
                points.push_back(id.cast<double>().cwiseProduct(stepScale) + oc);//cast类型转换转成double  cwiseProduct点乘
                    //比如    （1 200 30000）  点乘 {（1 1/100 1/10000） * 0.025}   +  origin + 半格m   =第一行 第二列 第三层  * resultion +origin +半格m =x方向多少m y方向多少m z方向多少m
                    //emmm这好像就是id->pos?
            }
            return;
        }
        inline void get_end_feasible_Surf(std::vector<Eigen::Vector3d> &points) const
        {
            points.reserve(end_feasible_surf.size());
            for (const Eigen::Vector3i &id : end_feasible_surf)
            {
                points.push_back(id.cast<double>().cwiseProduct(stepScale) + oc);//cast类型转换转成double  cwiseProduct点乘
                    //比如    （1 200 30000）  点乘 {（1 1/100 1/10000） * 0.025}   +  origin + 半格m   =第一行 第二列 第三层  * resultion +origin +半格m =x方向多少m y方向多少m z方向多少m
                    //emmm这好像就是id->pos?
            }
            return;
        }


        inline bool query(const Eigen::Vector3d &pos) const//occ
        {
            
            const Eigen::Vector3i id = ((pos - o) / scale).cast<int>();
            // std::cout<<"pos(2) "<<pos(2)  <<" id(2)  "<<id(2) <<std::endl;
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                // std::cout<<" return "<<voxels[id.dot(step)]<<std::endl;
                // return voxels[id.dot(step)];//OOC和Dilated都是true
                return voxel_begin[id.dot(step)];
            }
            else
            {   

                // return true;
                return false;
            }
        }

        inline bool query(const Eigen::Vector3i &id) const//occ
        {
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                // return voxels[id.dot(step)];
                return voxel_begin[id.dot(step)];
            }
            else
            {
                // return true;
                return false;
            }
        }

        inline Eigen::Vector3d posI2D(const Eigen::Vector3i &id) const
        {
            return id.cast<double>() * scale + oc;
        }

        inline Eigen::Vector3i posD2I(const Eigen::Vector3d &pos) const
        {
            return ((pos - o) / scale).cast<int>();
        }

        void init_GridNodeMap()
        {
            Eigen::Vector3i buffer_size=getSize();
            std::cout<<"buffer_size "<<buffer_size<<std::endl;
            GridNodeMap = new GridNodePtr ** [buffer_size[0]];
                for (int x = 0; x < buffer_size[0]; x++)
                {       
                    GridNodeMap[x] = new GridNodePtr * [buffer_size[1]];
                    for (int y = 0; y < buffer_size[1]; y++)
                    {
                        GridNodeMap[x][y] = new GridNodePtr [buffer_size[2]];
                        for( int z = 0; z < buffer_size[2];z++){
                            Vector3i tmpIdx_3d(x,y,z);
                            Eigen::Vector3d coord=posI2D(tmpIdx_3d);
                            GridNodeMap[x][y][z] = new GridNode(tmpIdx_3d,coord);

                        }
                    }
                }
        }
    void resetGrid(GridNodePtr ptr)
    {
        ptr->id = 0;
        ptr->cameFrom = NULL;
        ptr->gScore = inf;
        ptr->fScore = inf;
    }
    void resetUsedGrids()
{   Eigen::Vector3i buffer_size=getSize();
    for(int i=0; i < buffer_size[0] ; i++)
        for(int j=0; j < buffer_size[1] ; j++)
            for(int k=0; k < buffer_size[2] ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}
        void AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
    {   
        neighborPtrSets.clear();
        edgeCostSets.clear();
        // std::cout<<"777"<<std::endl;
        Eigen::Vector3i current_index=currentPtr->index;
       for(int num=0;num<voxel_connect_information_on_end_feasible[current_index.dot(step)].size();num++)
        // for(int num=0;num<voxel_connect_information_without_dilate[current_index.dot(step)].size();num++)
        {//这个连通信息里面可能有一些处于还没有按照机器人radius腐蚀的,所以要check一下在不在最终腐蚀后的可行集里
            // Eigen::Vector3i connect_neighbor=voxel_connect_information_without_dilate[current_index.dot(step)][num];
            // if(query_if_end_feasible_ok(connect_neighbor))//如果在可行域里
            //我在想给个障碍物如果比较矮但处于轮子不能碾过去的高度，通过法线滤波这个障碍物会不会被滤掉
            // {
                Eigen::Vector3i connect_neighbor=voxel_connect_information_on_end_feasible[current_index.dot(step)][num];
        
                int x = connect_neighbor(0);
                int y = connect_neighbor(1);
                int z = connect_neighbor(2);
                // std::cout<<"connect_neighbor "<<connect_neighbor<<std::endl;
                int dx = connect_neighbor(0)-current_index(0);
                int dy = connect_neighbor(1)-current_index(1);
                int dz = connect_neighbor(2)-current_index(2);
                GridNodePtr neighborPtr = GridNodeMap[x][y][z];
                neighborPtrSets.push_back(neighborPtr);
                double distance=sqrt(dx*dx+dy*dy+dz*dz);
                edgeCostSets.push_back(distance);
            // }
        }
        // std::cout<<"333"<<std::endl;
    }
        

                
         vector<Eigen::Vector3d> AstarnodeSearch(Eigen::Vector3d start, Eigen::Vector3d end)
    {    

        //index of start_point and end_point
        Vector3i start_idx = posD2I(start);
        Vector3i end_idx   = posD2I(end);
        Vector3i goalIdx = end_idx;

        GridNodePtr terminatePtr;
        GridNodePtr startPtr=new GridNode(start_idx, start);
        GridNodePtr endPtr=new GridNode(end_idx, end);
        std::multimap<double, GridNodePtr> openSet;std::multimap<double, GridNodePtr> closeSet;
        openSet.clear();closeSet.clear();
        GridNodePtr currentPtr  = NULL;
        GridNodePtr neighborPtr = NULL;
        startPtr -> gScore = 0;
        startPtr -> fScore = getHeu(startPtr,endPtr);   
        // startPtr -> id = 1; // 1--> open set, -1 --> closed set
        startPtr -> coord = start;
        openSet.insert( make_pair(startPtr -> fScore, startPtr) );
        // std::cout<<"111"<<std::endl;
        vector<GridNodePtr> neighborPtrSets;
        vector<double> edgeCostSets;
        while ( !openSet.empty() )
        {   
            // ROS_ERROR("openSet.size(),%d",openSet.size());   
            currentPtr=openSet.begin()->second;
            currentPtr->id=-1;// 1--> open set, -1 --> closed set 0-->没被拓展过
            openSet.erase(openSet.begin());
       
            if( currentPtr->index == goalIdx ){
                terminatePtr = currentPtr;   
                ROS_ERROR("break");    
                break;
            }
            AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);     
            // ROS_ERROR("neighborPtrSets.size(),%d",neighborPtrSets.size());    
                for(int i = 0; i < (int)neighborPtrSets.size(); i++)
                {
                    neighborPtr=neighborPtrSets[i];
                    if(neighborPtr -> id ==0){  
                        neighborPtr-> gScore=currentPtr->gScore+edgeCostSets[i];
                        neighborPtr-> fScore=neighborPtr-> gScore+getHeu(neighborPtr,endPtr);   
                        neighborPtr->cameFrom=currentPtr;
                        neighborPtr -> id =1;
                        openSet.insert( make_pair(neighborPtr -> fScore, neighborPtr) );
                        continue;
                    }
                    else if(neighborPtr -> id == 1){ 
                        if(neighborPtr-> gScore  >  currentPtr->gScore+edgeCostSets[i])
                        {
                        neighborPtr-> gScore=currentPtr->gScore+edgeCostSets[i];
                        neighborPtr-> fScore=neighborPtr-> gScore+getHeu(neighborPtr,endPtr); 
                        neighborPtr->cameFrom=currentPtr;
                        }
                        continue;
                    }
                    else{
                        continue;
                        }
                }      
        }
        vector<Vector3d> path;
        vector<GridNodePtr> gridPath;

        GridNodePtr stoptr=terminatePtr;
        gridPath.push_back(stoptr);
        Vector3i now_index,next_index;
        Vector3i bias;
        Vector3i last_bias(77,77,77);
        //尽管我已经把star同一朝向的给略掉了，但是在上扶梯和斜坡的时候还是出现了问题，因为坡度变化，离散化之后导致的
        while(stoptr->gScore!=0)
        {   now_index=stoptr->index;
            stoptr=stoptr->cameFrom;
            next_index=stoptr->index;
            if(stoptr->gScore==0)
                gridPath.push_back(stoptr);
            bias=now_index-next_index;
            if(bias!=last_bias)//这是因为传给DR wang的前端最好是一段一段，只有几个结点的，不然会慢。我之后会换成rrt*,采样在同一个并查集里面采
                gridPath.push_back(stoptr);
            last_bias=bias;
        }
        // std::cout<<"222"<<std::endl;
        for (auto ptr: gridPath){
            Vector3d upper(0,0,0.248+0.05);//这里应该放robot_height_min+0.05
            Vector3d tmp=ptr->coord+upper;//往上抬一格是为了后面minco的corridor
            path.push_back(tmp);
        }
        std::cout<<"path.size()"<<path.size()<<std::endl;
        reverse(path.begin(),path.end());

        return path;
        
    }

       double getHeu(GridNodePtr node1, GridNodePtr node2)
    {
        double h;
        auto now_3d_corrd=node1->coord;//    Eigen::Vector3d coord;
        auto end_3d_corrd=node2->coord;
        double dx=std::abs(now_3d_corrd(0)-end_3d_corrd(0));
        double dy=std::abs(now_3d_corrd(1)-end_3d_corrd(1));
        double dz=std::abs(now_3d_corrd(2)-end_3d_corrd(2));
        //3d 的 diag:   sqrt(3.0)*min_xyz+dxdydz-3min_xyz
        double min_xyz=std::min({dx, dy, dz});
        h=std::sqrt(3.0)*min_xyz+dx+dy+dz-3*min_xyz;

        h=h*1.000000001;
        return h;

    }


    };
}

#endif
