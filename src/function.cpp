// function.cpp
#include "function.h"

Vertex_set largest_component_nodes(10000);
// largest_component_nodes.max_load_factor(0.9);
Vertex_set removed_from_largest_nodes(200);

std::vector<connected_component> erosion_components(30);

// 全局变量frontier的节点集合
Vertex_set frontier_nodes(500);
NodeIndex_set frontier_indices(500);
// NodeID_set frontier_nodesID;
Vertex_set to_remove_frontier_nodes(500);

/******改掉，每个cluster添加target_node */
std::vector<std::pair<Vertex,Vertex_set>> frontier_clusters(50);

Graph G;
Graph G_h;

// 后续用std::unordered_map加速，但NodeID要改为int类型  ********************************
// std::map<NodeID, Vertex> IDtoV_map;
std::unordered_map<int, Vertex> indexToV_map;

std::vector<Vertex> path;

std::vector<room_state> rooms_vec(30);
std::map<room_state, Vertex> roomToV_map;

std::vector<room_state> rooms_adj(10);
Vertex_set unexplored_rooms(5);

std::vector<breadcrumb_state> breadcrumb_vec;

std::vector<Vertex_set> connected_components;

Vertex_set unaccessiable_nodes;

// 采样间隔
int sample_rate = 7;
int edge_id_counter = 0;
int update_map_times = 0;
int rebuild_map_frequency = 5;
int width = 0;
int height = 0;

double distance_gain = 1.0;
double information_gain = 0.01;
// double history_cost;
double robot_safety_value = 0.0;
int check_safety_depth = sample_rate + 3;

int frontier_threshold = 20;
// int frontier_threshold = 1;

// 上一帧用的什么网络，true表示高层，false表示低层
bool history_network = true;
double history_cost = 0.0;
int history_index = 0;
std::pair<double, double> history_pos;
double cost_diff_threshold = 3;

int occupied_grids_threshold = 3;
int cluster_info_threshold = 20;
double roomToCluster_dis_threshold = 4.0;
double roomToCluster_matchRate_threshold = 0.8;

int erosion_round = 3;
connected_component after_first_erosion_nodes;
// Vertex_set periphery_nodes;

std::chrono::duration<double> select_target_time;
std::chrono::duration<double> path_planning_time;

double euclid_distance(double x1, double y1, double x2, double y2){
    return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}

int get_index_in_map(double pos_x,double pos_y,const nav_msgs::OccupancyGrid::ConstPtr& msg){
    double abs_x = pos_x - msg->info.origin.position.x;
    double abs_y = pos_y - msg->info.origin.position.y;
    int grid_x = static_cast<int>(floor(abs_x/msg->info.resolution));
    int grid_y = static_cast<int>(floor(abs_y/msg->info.resolution));
    return grid_x + grid_y * width;
}

bool is_linear_connected(int grid_index_1, int grid_index_2, bool is_strict,const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    // int width = msg->info.width;
    // int height = msg->info.height;

    // 根据索引计算出对应的行和列
    int x1 = grid_index_1 % width;
    int y1 = grid_index_1 / width;
    int x2 = grid_index_2 % width;
    int y2 = grid_index_2 / width;

    int dx = std::abs(x2 - x1);
    int dy = -std::abs(y2 - y1);
    int sx = x1 < x2 ? 1 : -1;
    int sy = y1 < y2 ? 1 : -1;
    int err = dx + dy;  // error value e_xy



    while (true) {  // 循环绘制从(x1,y1)到(x2,y2)的线段
        if (x1 == x2 && y1 == y2) break;

        int e2 = 2 * err;
        if (e2 >= dy) {
            if (x1 == x2) break;
            err += dy;
            x1 += sx;
        }
        if (e2 <= dx) {
            if (y1 == y2) break;
            err += dx;
            y1 += sy;
        }

        // 检查当前栅格的状态
        int index = y1 * width + x1;
        if (index > 0 || index < msg->data.size()) {
            if ((is_strict && msg->data[index] != 0) || (!is_strict && msg->data[index] == 100))
            {
                return false; 
            }        
        }
        else
        {
            return false; 
        }
        
    }

    return true;  // 如果整个路径都是自由的，返回true
}





std::vector<Vertex> reconstruct_path(Vertex start, Vertex goal, const std::vector<Vertex>& predecessors) {
    std::vector<Vertex> path;
    Vertex current = goal;
    while (current != start) {
        path.push_back(current);
        current = predecessors[current];
    }
    path.push_back(start);  // 添加起点
    

    return path;
}

std::pair<geometry_msgs::Vector3, geometry_msgs::Quaternion> get_robot_position_in_map() {
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;

    try {
        transformStamped = tfBuffer.lookupTransform("map", "base_footprint", ros::Time(0), ros::Duration(4.0));
        return {transformStamped.transform.translation, transformStamped.transform.rotation};
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return {geometry_msgs::Vector3(), geometry_msgs::Quaternion()};
    }
}

std::pair<Vertex, double> get_robot_pose_in_graph(Graph& g, geometry_msgs::Vector3 pose, geometry_msgs::Quaternion quat,double ori_x,double ori_y,double resolution,const nav_msgs::OccupancyGrid::ConstPtr& msg){
    std::pair<double,double> abs_pose = {pose.x - ori_x, pose.y - ori_y};
    // ROS_INFO("robot pose is (%f,%f)",abs_pose.first,abs_pose.second);
    double x_gridmap_dot = abs_pose.first / resolution;
    double y_gridmap_dot = abs_pose.second / resolution;
    int x_gridmap = static_cast<int>(floor(x_gridmap_dot));
    int y_gridmap = static_cast<int>(floor(y_gridmap_dot));
    int robot_grid_index = x_gridmap + y_gridmap * width;


    double x_graph_dot = (x_gridmap_dot) / sample_rate;
    double y_graph_dot = (y_gridmap_dot) / sample_rate;
    // int x_graph_left = static_cast<int>(floor(x_graph_dot));
    // int y_graph_down = static_cast<int>(floor(y_graph_dot));
    // int x_graph_right = static_cast<int>(ceil(x_graph_dot));
    // int y_graph_up = static_cast<int>(ceil(y_graph_dot));
    int x_graph = static_cast<int>(round(x_graph_dot));
    int y_graph = static_cast<int>(round(y_graph_dot));

    // int robot_index_downleft = x_graph_left * sample_rate + y_graph_down * width *sample_rate;
    // int robot_index_downright = x_graph_right * sample_rate + y_graph_down * width *sample_rate;
    // int robot_index_upleft = x_graph_left * sample_rate + y_graph_up * width *sample_rate;
    // int robot_index_upright = x_graph_right * sample_rate + y_graph_up * width *sample_rate;

    // int robot_index_simi = x_graph * sample_rate + y_graph * width *sample_rate;
    int robot_index = x_graph * sample_rate + y_graph * width *sample_rate;
    
    
    Vertex robot_node;
    
    
    if (indexToV_map.count(robot_index) > 0 && g[indexToV_map[robot_index]].is_accessiable)
    {
        robot_node = indexToV_map[robot_index];

        
    }
    else{
        ROS_INFO("No robot node");
        
        int x_graph_left = static_cast<int>(floor(x_graph_dot));
        int y_graph_down = static_cast<int>(floor(y_graph_dot));
        // int x_graph_right = static_cast<int>(ceil(x_graph_dot));
        // int y_graph_up = static_cast<int>(ceil(y_graph_dot));
        robot_index = x_graph_left * sample_rate + y_graph_down * width * sample_rate;
        if (indexToV_map.count(robot_index) > 0 && g[indexToV_map[robot_index]].is_accessiable){
            robot_node = indexToV_map[robot_index];
        }
        else
        {
            robot_index += sample_rate;
            if (indexToV_map.count(robot_index) > 0 && g[indexToV_map[robot_index]].is_accessiable){
                robot_node = indexToV_map[robot_index];
            }
            else
            {
                robot_index += sample_rate * width;
                if (indexToV_map.count(robot_index) > 0 && g[indexToV_map[robot_index]].is_accessiable){
                    robot_node = indexToV_map[robot_index];
                }
                else
                {
                    robot_index -= sample_rate;
                    if (indexToV_map.count(robot_index) > 0 && g[indexToV_map[robot_index]].is_accessiable){
                        robot_node = indexToV_map[robot_index];
                    }
                    else
                    {
                        ROS_WARN("no robot node");
                    }
                }
                
            }
            
        }
    }
    tf2::Quaternion tfQuat;
    tf2::fromMsg(quat, tfQuat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tfQuat).getRPY(roll, pitch, yaw);
    // ROS_INFO("robot's position node is %ld, yaw is %f",robot_node,yaw);
    // int robot_index = x_grid + y_grid * width;
    return {robot_node,yaw};
}

void add_node(Graph& g, int grid_index, double abs_x, double abs_y, double origin_x, double origin_y) {
    // NodeID node_id = std::make_pair(x * sample_rate, y * sample_rate);
    Vertex v = boost::add_vertex(g);
    // IDtoV_map[node_id] = v;
    indexToV_map[grid_index] = v;
    // g[v].node_id = node_id;  
    g[v].pos = std::make_pair(abs_x + origin_x, abs_y + origin_y);
    g[v].grid_index = grid_index;
    g[v].frontier_size = 0;  // 初始化或者根据需要设置
    g[v].is_active = true;
    g[v].is_accessiable = true;
    largest_component_nodes.insert(v);
    frontier_nodes.insert(v);

    int y_grid = grid_index/width;
    int x_grid = grid_index%width;
    // ROS_INFO("add node index %d, (%d,%d)",grid_index,x_grid,y_grid);
}

void check_add_edges(Graph& g, Vertex cur_vertex, const nav_msgs::OccupancyGrid& msg) {
    // auto [cur_x, cur_y] = g[cur_vertex].node_id;
    int cur_index = g[cur_vertex].grid_index;
    int cur_x = cur_index % width;
    int cur_y = cur_index / width;
    // std::vector<NodeID> neighbors = {
    //     {cur_x + sample_rate, cur_y}, {cur_x - sample_rate, cur_y},
    //     {cur_x, cur_y + sample_rate}, {cur_x, cur_y - sample_rate},
    //     {cur_x + sample_rate, cur_y + sample_rate}, {cur_x - sample_rate, cur_y + sample_rate},
    //     {cur_x + sample_rate, cur_y - sample_rate}, {cur_x - sample_rate, cur_y - sample_rate}
    // };
    // std::vector<int> neighbors = {
    //     cur_x + sample_rate + cur_y * width , 
    //     cur_x - sample_rate + cur_y * width,
    //     cur_x + (cur_y + sample_rate) * width, 
    //     cur_x + (cur_y - sample_rate) * width,
    //     cur_x + sample_rate + (cur_y + sample_rate) * width, 
    //     cur_x - sample_rate + (cur_y + sample_rate) * width,
    //     cur_x + sample_rate + (cur_y - sample_rate) * width, 
    //     cur_x - sample_rate + (cur_y - sample_rate) * width
    // };
    // std::vector<NodeID> neighbors = filter_existing_neighbors(g,cur_vertex,true);

    // for (NodeID neighbor_id : neighbors) {
    //     auto [nx,ny] = neighbor_id;
    //     if (IDtoV_map.count(neighbor_id) > 0 ){
    //         Vertex neighbor_vertex = IDtoV_map[neighbor_id];
    // for (int neighbor_index : neighbors) {
    for (int x_dir = -1; x_dir <= 1; x_dir++)
    {
        for (int y_dir = -1; y_dir <= 1; y_dir++)
        {
            if (x_dir == 0 && y_dir == 0)
            {
                continue;
            }
            int check_node_x = cur_x + sample_rate * x_dir;
            int check_node_y = cur_y + sample_rate * y_dir;
            int check_node_index = check_node_x + check_node_y * width;
            // ROS_INFO("add edge %d, %d",x_dir,y_dir);

            if (indexToV_map.count(check_node_index) > 0){
                Vertex neighbor_vertex = indexToV_map[check_node_index];
                // ROS_INFO("add edge is check node exist");
                if (!boost::edge(cur_vertex, neighbor_vertex, g).second)
                {
                    // ROS_INFO("add edge is no edge");
                    bool is_add_edge = true;
                    // ROS_INFO("%d, %d",diff_x,diff_y);
                    for (int i = 1; i <= sample_rate; ++i) {
                        int check_x = cur_x + i * x_dir;
                        int check_y = cur_y + i * y_dir;
                        int check_index = check_y * msg.info.width + check_x;
                        if (msg.data[check_index] != 0) {
                            is_add_edge = false;
                            break;
                        }
                        for (int j = -sample_rate/2; j < sample_rate/2; j++)
                        {
                            check_x = (cur_x + j*y_dir) + i * x_dir;
                            check_y = (cur_y - j*x_dir) + i * y_dir;
                            check_index = check_y * msg.info.width + check_x;
                            if (msg.data[check_index] == 100) {
                                is_add_edge = false;
                                break;
                            }
                        }
                        
                    }
                    if (is_add_edge) {
                        Edge edge;
                        bool inserted;
                        boost::tie(edge, inserted) = boost::add_edge(cur_vertex, neighbor_vertex, G);
                        double distance = std::sqrt(std::pow(g[cur_vertex].pos.first - g[neighbor_vertex].pos.first, 2) +
                                                    std::pow(g[cur_vertex].pos.second - g[neighbor_vertex].pos.second, 2));
                        
                        if (inserted) {
                            boost::put(boost::edge_weight, G, edge, distance);
                            // ROS_INFO("edge weight is %f",distance);
                            // add_edge_visualization(cur_vertex, neighbor_vertex,origin_x,origin_y, marker_array);
                        }
                    }
                }
            
            }
            else
            {
                // ROS_INFO("no neibor index %d,(%d,%d), cur_x = %d, cur_y = %d, cur_index = %d",check_node_index,check_node_x,check_node_y,cur_x,cur_y,cur_index);
            }
        }
    }
}

void add_update_node(Graph& g, int grid_index, double abs_x, double abs_y, double origin_x, double origin_y, const nav_msgs::OccupancyGrid& msg) {
    // NodeID node_id = std::make_pair(x * sample_rate, y * sample_rate);
    // if (IDtoV_map.count(node_id) == 0)

    Vertex v = boost::add_vertex(G);
    // IDtoV_map[node_id] = v; 
    indexToV_map[grid_index] = v; 
    // g[v].node_id = node_id;
    g[v].pos = std::make_pair(abs_x + origin_x, abs_y + origin_y);
    g[v].frontier_size = 0;  // 初始化或者根据需要设置
    g[v].is_active = true;
    g[v].is_accessiable = true;
    g[v].grid_index = grid_index;//沃日
    largest_component_nodes.insert(v);
    // ROS_INFO("add update node ");
    check_add_edges(g, v, msg); 
    frontier_nodes.insert(v);

    int y_grid = grid_index/width;
    int x_grid = grid_index%width;
    // ROS_INFO("add update node index %d, (%d,%d)",grid_index,x_grid,y_grid);

} 


void remove_non_largest_component(Graph& g) {
    // auto start_remove_time = std::chrono::high_resolution_clock::now();
    for (auto& vertex : removed_from_largest_nodes) {
        largest_component_nodes.insert(vertex);
        frontier_nodes.insert(vertex);
        g[vertex].is_accessiable = true;
    }
    removed_from_largest_nodes.clear();
    
    // auto after_connected_time = std::chrono::high_resolution_clock::now();

    std::vector<int> component(boost::num_vertices(g));
    int num = boost::connected_components(g, &component[0]);


    // 计算每个连通分量的大小并追踪最大连通分量
    std::map<int, int> component_size;
    int max_size = 0;
    int largest_comp_id = -1;

    for (int comp_id : component) {
        int size = ++component_size[comp_id];
        if (size > max_size) {
            max_size = size;
            largest_comp_id = comp_id;
        }
    }

    // 移除不在最大连通分量中的顶点
    for (size_t i = 0; i < component.size(); ++i) {
        if (component[i] != largest_comp_id) {
            g[i].is_accessiable = false;
            largest_component_nodes.erase(i);
            frontier_nodes.erase(i);
            removed_from_largest_nodes.insert(i);
        }
    }
    // auto end_remove_time = std::chrono::high_resolution_clock::now();

    // std::chrono::duration<double> duration = end_remove_time - start_remove_time;
    // std::chrono::duration<double> connected_components_time = after_connected_time - start_remove_time;
    // double connected_components_time_rate = 100 * connected_components_time / duration;
    // std::chrono::duration<double> process_time = end_remove_time - after_connected_time;
    // double process_time_rate = 100 * process_time / duration;
    // std::cout << "Time taken by function: "
    //         << std::fixed << std::setprecision(4) << duration.count() << " seconds" << std::endl;
    // std::cout << "Time taken by build graph: " << std::fixed << std::setprecision(4) 
    //         << connected_components_time.count() << " seconds, rate is " << connected_components_time_rate << "%" << std::endl;
    // std::cout << "Time taken by remove largest: " << std::fixed << std::setprecision(4) 
    //         << process_time.count() << " seconds, rate is " << process_time_rate << "%" << std::endl;

}

// 函数：检查前沿
void check_frontier(Graph& g, const nav_msgs::OccupancyGrid& data) {
    // vertex_iter vi, vend;
    // 可以优化一下，只遍历largest中的节点
    // for (boost::tie(vi, vend) = vertices(g); vi != vend; ++vi) 
    
    to_remove_frontier_nodes.clear();
    for (auto& v : frontier_nodes) 
    {
        // Vertex v = *vi;
        if (degree(v, g) < 8 && g[v].is_active == true) {
            // 这里是为了建立本该建立的边，当两个节点被建立且当时不连通而没有第一时间建立边，而后面连通了的时候将不再更新，所以在这里趁机检测一下
            int unknown_num = 0;
            check_add_edges(g,v,data);
            if (degree(v, g) == 8)
            {
                // g[v].frontier_size = 0;
                to_remove_frontier_nodes.insert(v);
            }
            else
            {
                
                // NodeID detect_nodeID = g[v].node_id;
                // auto [x,y] = detect_nodeID;
                int cur_index = g[v].grid_index;
                // int cur_x = cur_index % width;
                // int cur_y = cur_index / width;
                int check_depth = sample_rate + 3;
                bool is_next_check = true;
                int check_occ_number = 0;
                for (int d = 1; d <= check_depth; d++)
                {
                    if (is_next_check)
                    {
                        // for (int i = -d + 1; i <= d - 1; i++)
                        for (int i = -d; i <= d; i++)
                        {
                            int check_index_down = cur_index + i - d * width;
                            int check_index_up = cur_index + i + d * width;
                            int check_index_left = cur_index + d + i * width;
                            int check_index_right = cur_index - d + i * width;
                            if (data.data[check_index_down] == 100 || data.data[check_index_up] == 100 ||data.data[check_index_left] == 100 ||data.data[check_index_right] == 100)
                            {
                                if (check_occ_number < occupied_grids_threshold)
                                {
                                    check_occ_number++;
                                }
                                else
                                {
                                    is_next_check = false;
                                } 
                            }
                            if (data.data[check_index_down] == -1)
                            {
                                unknown_num++;
                            }
                            if (data.data[check_index_up] == -1)
                            {
                                unknown_num++;
                            }if (data.data[check_index_left] == -1)
                            {
                                unknown_num++;
                            }if (data.data[check_index_right] == -1)
                            {
                                unknown_num++;
                            }
                        }
                    }
                    else
                    {
                        break;
                    }
                }
            }
            g[v].frontier_size = unknown_num;
            if (unknown_num == 0)
            {
                // frontier_nodes.erase(v);
                to_remove_frontier_nodes.insert(v);
            }
            
        }
        
        
            // g[v].color = (unknown_num > 10) ? "green" : "blue";
        else {
            g[v].frontier_size = 0;
            // frontier_nodes.erase(v);
            to_remove_frontier_nodes.insert(v);
            // g[v].color = "blue";
        }
    }
    for (auto& v : to_remove_frontier_nodes){
        frontier_nodes.erase(v);
    }
    // frontier_nodesID.clear();
    // for (auto& v : frontier_nodes){
    //     frontier_nodesID.insert(g[v].node_id);
    // }
    frontier_indices.clear();
    for (auto& v : frontier_nodes){
        frontier_indices.insert(g[v].grid_index);
        // std::cout << v << " ";
    }
}

Vertex_set BFS_search(Graph& g, Vertex& v, Vertex_set& visitd_frontier){
    Vertex_set cluster;
    std::queue<Vertex> queue;
    visitd_frontier.insert(v);
    queue.push(v);
    while (!queue.empty()) {
        Vertex current = queue.front();
        queue.pop();
        cluster.insert(current);

        // 遍历所有邻接节点
        auto adjacent_range = boost::adjacent_vertices(current, g);
        for (auto it = adjacent_range.first; it != adjacent_range.second; ++it) {
            Vertex neighbor = *it;
            if (frontier_nodes.count(neighbor) > 0 && visitd_frontier.count(neighbor) == 0)
            {
                visitd_frontier.insert(neighbor);
                if (g[neighbor].frontier_size > frontier_threshold)
                {
                    // cluster.insert(neighbor);
                    queue.push(neighbor);
                }
                
                
            }
            
            // if (visited.find(neighbor) == visited.end()) {
            //     visited.insert(neighbor);
            //     queue.push(neighbor);
            // }
        }
    }
    return cluster;
}

void BFS_search(Graph& g, Vertex& start, Vertex_set& limited_set, std::unordered_set<Vertex>& visited, Vertex_set& component){
    std::queue<Vertex> queue;
    queue.push(start);
    visited.insert(start);
    

    while (!queue.empty()) {
        Vertex v = queue.front();

        component.insert(v);
        queue.pop();
        
        // 遍历v的邻居
        auto adjacent_range = boost::adjacent_vertices(v, g);
        for (auto it = adjacent_range.first; it != adjacent_range.second; ++it) {
            Vertex neighbor = *it;
            if (limited_set.find(neighbor) != limited_set.end() && visited.find(neighbor) == visited.end()) {
                visited.insert(neighbor);
                component.insert(neighbor);
                queue.push(neighbor);
            }
        }
    }
}



void cluster_frontier(Graph& g){
    Vertex_set temp_cluster;
    Vertex_set visitd_frontier;

    frontier_clusters.clear();
    // int cluster_index = 0;

    for (auto v : frontier_nodes){
        if(visitd_frontier.count(v) > 0 || g[v].frontier_size <= frontier_threshold){
            continue;
        }
        else{
            std::pair<Vertex,Vertex_set> temp_cluster;
            
            temp_cluster.second = BFS_search(g,v,visitd_frontier);

            if (temp_cluster.second.size() > 1)
            // if (temp_cluster.second.size() > 0)
            {
                int node_number = temp_cluster.second.size();
                double total_x = 0.0;
                double total_y = 0.0;
                std::pair<double,double> geometric_center = {0.0,0.0};
                // int information_volume = 0;
                for (Vertex frontier_node: temp_cluster.second)
                {
                    total_x += g[frontier_node].pos.first;
                    total_y += g[frontier_node].pos.second;
                    // information_volume += g[frontier_node].frontier_size;
                }
                geometric_center.first = total_x / node_number;
                geometric_center.second = total_y / node_number;
                
                double nearest_distance = std::numeric_limits<double>::max();

                int information_value = 0;

                for (Vertex frontier_node: temp_cluster.second)
                {
                    information_value += g[frontier_node].frontier_size;
                    double distance = std::pow(g[frontier_node].pos.first - geometric_center.first,2) + 
                        std::pow(g[frontier_node].pos.second - geometric_center.second,2);
                    if (distance < nearest_distance)
                    {
                        temp_cluster.first = frontier_node;
                        nearest_distance = distance;
                    }
                }
                if (information_value > cluster_info_threshold)
                {
                    frontier_clusters.push_back(std::move(temp_cluster));
                }
                
                
            }
        }
    }
    // for (auto& cluster : frontier_clusters)
    // {
    //     std::cout << "cluster" << std::endl;
    //     for (auto& v : cluster)
    //     {
    //         std::cout << v << " ";
    //     }
 
}






std::pair<double,double> low_level_network_planner(Graph& g,Graph& g_h,geometry_msgs::Vector3 robot_pos,Vertex robot_node, double robot_yaw,const nav_msgs::OccupancyGrid::ConstPtr& msg){
    // std::unordered_map<Vertex,double> path_cost;
    // std::unordered_map<Vertex,double> turn_cost;
    // std::unordered_map<Vertex,double> info_gain;
    // std::unordered_map<Vertex,double> direction;

    
    auto start = std::chrono::high_resolution_clock::now();

    Vertex_set targetNode_set;
    Vertex best_goal;
    
    std::pair<double,double> totalCost_and_navYaw = {std::numeric_limits<double>::max(),0.0};

    // bool is_no_accessiable_frontier = true;
    // for (std::pair<Vertex,Vertex_set>& cluster: frontier_clusters)
    // {
    //     if (unaccessiable_nodes.find(cluster.first) == unaccessiable_nodes.end())
    //     {
    //         is_no_accessiable_frontier = false;
    //     }
    // }
    // if (is_no_accessiable_frontier)
    // {
    //     /* code */
    // }
    
    

    
    bool is_history_exist = false;
    bool is_curNode_history = false;
    Vertex history_target_node;
    for (std::pair<Vertex,Vertex_set>& cluster: frontier_clusters)
    {
        Vertex target_node = cluster.first;
        if (unaccessiable_nodes.find(cluster.first) != unaccessiable_nodes.end())
        {
            continue;
        }
        
        if (!history_network && euclid_distance(g[target_node].pos.first,g[target_node].pos.second,history_pos.first,history_pos.second) < 1.5 && is_linear_connected(g[target_node].grid_index,history_index,true,msg))
        {
            is_history_exist = true;
            is_curNode_history = true;
            history_target_node = target_node;
            // ROS_WARN("check low-network history target");
        }
        
        // 定义启发式和目标
        // Vertex goal = v2;
        // ROS_WARN("success run0");
        EuclideanHeuristic heuristic(g, target_node);
        astar_goal_visitor visitor(target_node);

        std::vector<Vertex> predecessors(boost::num_vertices(g));
        std::vector<double> distances(boost::num_vertices(g), std::numeric_limits<double>::max());

        // ROS_WARN("success run1");
        try {
            boost::astar_search(g, robot_node, heuristic,
                                boost::visitor(visitor).
                                predecessor_map(boost::make_iterator_property_map(predecessors.begin(), boost::get(boost::vertex_index, g))).
                                distance_map(boost::make_iterator_property_map(distances.begin(), boost::get(boost::vertex_index, g))));
        } catch (astar_goal_visitor::found_goal fg) {
            // 处理找到目标的情况
            std::cout << "Goal reached!" << std::endl;
        }
        // ROS_WARN("success run2");
        


        // ROS_INFO("ROBOT node is %ld",robot_node);
        // ROS_INFO("next next node is %ld",next_next_node);

        int information_volume = 0;
        for (Vertex node: cluster.second)
        {
            information_volume += g[node].frontier_size;
        }
        
        
        double pathlength_cost = distances[target_node];
        // double total_cost = distance_gain * pathlength_cost - information_gain * information_volume;
        double total_cost = distance_gain * pathlength_cost;
        

        if (is_curNode_history)
        {
            history_cost = total_cost;
            // 使用这个函数构建路径
            is_curNode_history = false;
            if (total_cost - totalCost_and_navYaw.first < cost_diff_threshold)
            {
                totalCost_and_navYaw.first = total_cost;
                // totalCost_and_navYaw.second = diff_yaw;
                best_goal = target_node;
                path = reconstruct_path(robot_node, best_goal, predecessors); 
            }
            else
            {
                // ROS_ERROR("jump start high-network recurrent error");
            }
            
        }
        if (!is_curNode_history && total_cost < totalCost_and_navYaw.first)
        {
            if (is_history_exist)
            {
                if (history_cost - total_cost > cost_diff_threshold)
                {
                    // 使用这个函数构建路径
                    totalCost_and_navYaw.first = total_cost;
                    // totalCost_and_navYaw.second = diff_yaw;
                    best_goal = target_node;
                    path = reconstruct_path(robot_node, best_goal, predecessors); 
                    // ROS_ERROR("jump process low-network recurrent error");
                }

            }
            else
            {
                // 使用这个函数构建路径
                totalCost_and_navYaw.first = total_cost;
                // totalCost_and_navYaw.second = diff_yaw;
                best_goal = target_node;
                path = reconstruct_path(robot_node, best_goal, predecessors); 
            }
        }
    }

    history_network = false;
    // history_cost = totalCost_and_navYaw.first;
    history_index = g[best_goal].grid_index;
    history_pos = g[best_goal].pos;
    
    

    auto after_select_target = std::chrono::high_resolution_clock::now();
    select_target_time = after_select_target - start;
    
    std::reverse(path.begin(), path.end());  // 反转路径以从起点开始

    

    // 消融实验参数
    dwa_goal_dir_x = g[best_goal].pos.first - g[path[path.size()-2]].pos.first;
    dwa_goal_dir_y = g[best_goal].pos.second - g[path[path.size()-2]].pos.second;
    dwa_goal_pos_x = g[best_goal].pos.first;
    dwa_goal_pos_y = g[best_goal].pos.second;
    
    return totalCost_and_navYaw;
}



