#ifndef FUNCTION_H
#define FUNCTION_H

#include <ros/ros.h>
#include <boost/graph/adjacency_list.hpp>  // 包含 BGL 图结构
#include <boost/graph/graph_traits.hpp>    // 包含图的特性和访问方法
#include "nav_msgs/OccupancyGrid.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <map>
#include <utility>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath> 
#include <cstdlib>
#include <vector>
#include <array>
#include <boost/graph/connected_components.hpp>
#include <algorithm>
#include <iostream>
#include <boost/graph/graphviz.hpp>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <boost/graph/filtered_graph.hpp>
#include <unordered_set>
#include <queue>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <limits>
#include <boost/graph/astar_search.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>


struct VertexProperties {
    int frontier_size = 0;
    int grid_index = -1;
    std::pair<double, double> pos;
    bool is_active = true;
    bool is_accessiable = true;
    int erosion_layer = 0;
    bool is_room_node = false;
    bool is_breadcrumb_node = false;
    bool is_unexplored = false;
};
typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperties, EdgeWeightProperty> Graph;

typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;

typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
typedef boost::graph_traits<Graph>::edge_iterator edge_iter;

// 顶点过滤器
struct VertexFilter {
    std::unordered_set<Vertex> node_set;

    VertexFilter() = default;  // 默认构造函数
    VertexFilter(const std::unordered_set<Vertex>& nodes) : node_set(nodes) {}

    
    bool operator()(const Vertex& v) const {
        return node_set.find(v) != node_set.end();
    }
};

// 边过滤器，这里我们只保留两端都在集合中的边
struct EdgeFilter {
    std::shared_ptr<Graph> g;
    std::unordered_set<Vertex> node_set;

    // 默认构造函数
    EdgeFilter() : g(std::make_shared<Graph>()), node_set() {}

    // 正常使用的构造函数
    EdgeFilter(std::shared_ptr<Graph> graph, const std::unordered_set<Vertex>& nodes)
        : g(graph), node_set(nodes) {}

    bool operator()(const Edge& e) const {
        if (!g) return false;  // 如果图是默认构造的，可能需要这样的检查
        auto u = source(e, *g), v = target(e, *g);
        return node_set.find(u) != node_set.end() && node_set.find(v) != node_set.end();
    }
};


typedef boost::filtered_graph<Graph, EdgeFilter, VertexFilter> subgraph;

typedef boost::graph_traits<subgraph>::vertex_iterator subgraph_vertex_iter;
typedef boost::graph_traits<subgraph>::edge_iterator subgraph_edge_iter;

typedef std::unordered_set<Vertex> Vertex_set;
typedef std::unordered_set<int> NodeIndex_set;

typedef std::pair<bool,Vertex_set> connected_component;
typedef std::pair<bool,std::array<double,2>> room_state;

typedef std::pair<std::array<double,2>,std::unordered_set<Vertex>> breadcrumb_state;


typedef boost::graph_traits<Graph>::adjacency_iterator adjacency_iterator;

extern Vertex_set largest_component_nodes;
// largest_component_nodes.max_load_factor(0.9);
extern Vertex_set removed_from_largest_nodes;

extern std::vector<connected_component> erosion_components;

// 全局变量frontier的节点集合
extern Vertex_set frontier_nodes;
extern NodeIndex_set frontier_indices;
// NodeID_set frontier_nodesID;
extern Vertex_set to_remove_frontier_nodes;

/******改掉，每个cluster添加target_node */
extern std::vector<std::pair<Vertex,Vertex_set>> frontier_clusters;

extern std::vector<Vertex_set> connected_components;
extern Vertex_set unaccessiable_nodes;

extern Graph G;
extern Graph G_h;

// 后续用std::unordered_map加速，但NodeID要改为int类型  ********************************
// std::map<NodeID, Vertex> IDtoV_map;
extern std::unordered_map<int, Vertex> indexToV_map;

extern std::vector<Vertex> path;

extern std::vector<room_state> rooms_vec;
extern std::map<room_state, Vertex> roomToV_map;

extern std::vector<room_state> rooms_adj;
extern Vertex_set unexplored_rooms;

extern std::vector<breadcrumb_state> breadcrumb_vec;

// 采样间隔
extern int sample_rate;
extern int edge_id_counter;
extern int update_map_times;
extern int rebuild_map_frequency;
extern int width;
extern int height;

extern double distance_gain;
extern double information_gain;
// double history_cost;
extern double robot_safety_value;
extern int check_safety_depth;

extern int frontier_threshold;
// int frontier_threshold = 1;

// 上一帧用的什么网络，true表示高层，false表示低层
extern bool history_network;
extern double history_cost;
extern int history_index;
extern std::pair<double, double> history_pos;
extern double cost_diff_threshold;

extern int occupied_grids_threshold;
extern int cluster_info_threshold;
extern double roomToCluster_dis_threshold;
extern double roomToCluster_matchRate_threshold;

extern int erosion_round;

extern double velocity_step;
extern double turn_step;

extern double safety_gain;
extern double goal_gain;

extern std::chrono::duration<double> select_target_time;
extern std::chrono::duration<double> path_planning_time;

//消融实验变量
extern bool is_single_low_network;

extern double dwa_goal_dir_x; // 目标方向的 x 分量
extern double dwa_goal_dir_y; // 目标方向的 y 分量
extern double dwa_goal_pos_x; // 目标的 x 坐标
extern double dwa_goal_pos_y; // 目标的 y 坐标

class EuclideanHeuristic : public boost::astar_heuristic<Graph, double> {
public:
    EuclideanHeuristic(const Graph& g, Vertex goal) : g(g), goal(goal) {}


    double operator()(Vertex v) const {

        const std::pair<double, double>& p1 = g[v].pos;
        const std::pair<double, double>& p2 = g[goal].pos;
        double dx = p1.first - p2.first;
        double dy = p1.second - p2.second;
        double euclidean_distance = std::sqrt(dx * dx + dy * dy);

        return euclidean_distance;
    }

private:
    const Graph& g; 
    Vertex goal;
};

class astar_goal_visitor : public boost::default_astar_visitor {
public:
    astar_goal_visitor(Vertex goal) : goal(goal) {}

    void examine_vertex(Vertex u, const Graph& g) {
        if (u == goal)
            throw found_goal();
    }

    class found_goal {};  // Exception to stop the search when goal is reached
private:
    Vertex goal;
};

double euclid_distance(double x1, double y1, double x2, double y2);

int get_index_in_map(double pos_x,double pos_y,const nav_msgs::OccupancyGrid::ConstPtr& msg);

bool is_linear_connected(int grid_index_1, int grid_index_2, bool is_strict,const nav_msgs::OccupancyGrid::ConstPtr& msg);



std::vector<Vertex> reconstruct_path(Vertex start, Vertex goal, const std::vector<Vertex>& predecessors);

std::pair<geometry_msgs::Vector3, geometry_msgs::Quaternion> get_robot_position_in_map();

std::pair<Vertex, double> get_robot_pose_in_graph(Graph& g, geometry_msgs::Vector3 pose, geometry_msgs::Quaternion quat,double ori_x,double ori_y,double resolution,const nav_msgs::OccupancyGrid::ConstPtr& msg);

void add_node(Graph& g, int grid_index, double abs_x, double abs_y, double origin_x, double origin_y);

void check_add_edges(Graph& g, Vertex cur_vertex, const nav_msgs::OccupancyGrid& msg);

void add_update_node(Graph& g, int grid_index, double abs_x, double abs_y, double origin_x, double origin_y, const nav_msgs::OccupancyGrid& msg);

void add_LTNnodes_visualization(Graph& g, visualization_msgs::MarkerArray& marker_array, bool is_periphery);

void add_allnodes_visualization(Graph& g, visualization_msgs::MarkerArray& marker_array, bool is_low_level_network);

void add_alledges_visualization(Graph& g, visualization_msgs::MarkerArray& marker_array, bool is_low_level_network);

void add_path_visualization(Graph& g, std::vector<Vertex> path, visualization_msgs::MarkerArray& marker_array);

void add_room_visualization(Graph& g, std::vector<room_state> rooms_vector, visualization_msgs::MarkerArray& marker_array);

void add_frontier_visualization(Graph& g, visualization_msgs::MarkerArray& marker_array);

void publishGoalMarker(const ros::Publisher& marker_pub, std::pair<double, double>& target_pose);

void remove_non_largest_component(Graph& g);

void check_frontier(Graph& g, const nav_msgs::OccupancyGrid& data);

Vertex_set BFS_search(Graph& g, Vertex& v, Vertex_set& visitd_frontier);

void cluster_frontier(Graph& g);







std::pair<double,double> low_level_network_planner(Graph& g,Graph& g_h,geometry_msgs::Vector3 robot_pos,Vertex robot_node, double robot_yaw,const nav_msgs::OccupancyGrid::ConstPtr& msg);



#endif // FUNCTION_H
