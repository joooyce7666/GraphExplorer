#include "function.h"




std::vector<int8_t> history_map;



ros::Subscriber map_sub;
ros::Publisher vel_pub;
ros::Publisher marker_pub;
ros::Publisher marker_pub_LTN;
geometry_msgs::Twist target_vel;
geometry_msgs::Twist current_vel;
double velocity_step = 0.2;
double turn_step = 0.32;

double safety_gain = 1.8;
double goal_gain = 1.0;

ros::Publisher goal_pub;



double dwa_goal_dir_x; // 目标方向的 x 分量
double dwa_goal_dir_y; // 目标方向的 y 分量
double dwa_goal_yaw;
tf2::Quaternion dwa_goal_quaternion;
double dwa_goal_pos_x; // 目标的 x 坐标
double dwa_goal_pos_y; // 目标的 y 坐标

ros::Publisher dwa_goal_pub;






void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    
    
    // 清空所有之前的标记
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::MarkerArray marker_array_LTN;
    
    // bool isMapSizeChange = false;
    if (width != msg->info.width || height != msg->info.height)
    {
        // isMapSizeChange = true;
        update_map_times = 0; //rebuild map
        ROS_INFO("map size change");
    }
    
    // 获取地图的分辨率、宽度和高度
    width = msg->info.width;
    height = msg->info.height;

    double resolution = msg->info.resolution;
    // 地图原点
    double origin_x = msg->info.origin.position.x;
    double origin_y = msg->info.origin.position.y;
    double origin_z = msg->info.origin.position.z;

    // 获取地图数据
    const std::vector<int8_t>& map_data = msg->data;
    auto [cur_pos, cur_quat] = get_robot_position_in_map();
    

    // 宽度和高度的整数除法
    int width_in_graph = width / sample_rate;
    int height_in_graph = height / sample_rate;

    std::pair<double,double> cost_yaw;

    std::chrono::duration<double> elapsed;
    std::chrono::duration<double> update_graph_time;
    std::chrono::duration<double> detect_frontier_time;
    std::chrono::duration<double> cluster_frontier_time;
    std::chrono::duration<double> generat_highnetwork_time;
    auto start = std::chrono::high_resolution_clock::now();

    // if(update_map_times % update_map_frequency == 0 || sample_rate == 7)



    G_h.clear();
    unaccessiable_nodes.clear();
    if(update_map_times % rebuild_map_frequency == 0)
    {
    // 清空图和位置字典
        G.clear();
        // pos.clear();
        // IDtoV_map.clear();  // 清空节点到顶点的映射
        indexToV_map.clear();  // 清空节点到顶点的映射
        largest_component_nodes.clear();
        removed_from_largest_nodes.clear();
        frontier_nodes.clear();
        // 遍历所有要采样的栅格
        for (int y = 0; y < height_in_graph; ++y) {  // 外层循环遍历行
            for (int x = 0; x < width_in_graph; ++x) {  // 内层循环遍历列
                // 计算一维数组中的索引
                int index = y * sample_rate * width + x * sample_rate;
                if (map_data[index] == 0) {
                    // 以每个栅格的中心位置作为其坐标
                    double occmap_x = x * sample_rate * resolution + 0.5 * resolution;
                    double occmap_y = y * sample_rate * resolution + 0.5 * resolution;
                    add_node(G,index,occmap_x,occmap_y,origin_x,origin_y);   
                }
            }
        }
        
    
        std::pair<vertex_iter, vertex_iter> vp;
        // 遍历所有顶点
        for (vp = boost::vertices(G); vp.first != vp.second; ++vp.first) {
            Vertex v = *vp.first;  // 当前节点   
            check_add_edges(G,v, *msg);  // 确保这个函数可以接受这些参数
        }
        ROS_INFO("Rebuild graph");
        // size_t num_vertices_in_graph = num_vertices(G);
        // size_t num_entries_in_map = IDtoV_map.size();
        // if (num_vertices_in_graph != num_entries_in_map)
        // {
        //     ROS_WARN("vertex number is %ld, mapping number is %ld",num_vertices_in_graph,num_entries_in_map);
        // }
        // else
        // {
        //     ROS_INFO("vertex number is %ld, mapping number is %ld",num_vertices_in_graph,num_entries_in_map);
        // }
    }
    // 更新这里最好还优化一下更快一点，只更新机器人附近的节点以及度小于8的节点及其周围(经测试，不需优化，这里占用的计算资源几乎忽略不计)
    else{
        for (int y = 0; y < height_in_graph; ++y) {  // 外层循环遍历行
            for (int x = 0; x < width_in_graph; ++x) {  // 内层循环遍历列
                // 计算一维数组中的索引
                int index = y * sample_rate * width + x * sample_rate;
                // 获取当前栅格的概率值
                int current_value = map_data[index];
                int history_value = history_map[index];
                if (current_value != history_value){
                    if (current_value == 0)
                    {
                        // NodeID cur_ID = std::make_pair(x * sample_rate, y * sample_rate);  
                        // if(IDtoV_map.count(cur_ID) == 0){
                        //     double occmap_x = x * sample_rate * resolution + 0.5 * resolution;
                        //     double occmap_y = y * sample_rate * resolution + 0.5 * resolution;
                        //     add_update_node(G,x,y,occmap_x,occmap_y,origin_x,origin_y, *msg); 
                        // }
                        if(indexToV_map.count(index) == 0){
                            double occmap_x = x * sample_rate * resolution + 0.5 * resolution;
                            double occmap_y = y * sample_rate * resolution + 0.5 * resolution;
                            add_update_node(G,index,occmap_x,occmap_y,origin_x,origin_y, *msg); 
                            // check_add_edges(G,indexToV_map[index],*msg);
                        }
                        else
                        {
                            // G[IDtoV_map[cur_ID]].is_active = true;
                            // G[IDtoV_map[cur_ID]].is_accessiable = true;
                            // check_add_edges(G,IDtoV_map[cur_ID],*msg);
                            // largest_component_nodes.insert(IDtoV_map[cur_ID]);
                            // frontier_nodes.insert(IDtoV_map[cur_ID]);
                            G[indexToV_map[index]].is_active = true;
                            G[indexToV_map[index]].is_accessiable = true;
                            check_add_edges(G,indexToV_map[index],*msg);
                            largest_component_nodes.insert(indexToV_map[index]);
                            frontier_nodes.insert(indexToV_map[index]);
                        }
                        
                    }
                    if (history_value == 0) {
                        // NodeID discard_nodeID = std::make_pair(x * sample_rate, y * sample_rate);
                        // G[IDtoV_map[discard_nodeID]].is_active = false;
                        // G[IDtoV_map[discard_nodeID]].is_accessiable = false;
                        // largest_component_nodes.erase(IDtoV_map[discard_nodeID]);
                        // frontier_nodes.erase(IDtoV_map[discard_nodeID]);
                        G[indexToV_map[index]].is_active = false;
                        G[indexToV_map[index]].is_accessiable = false;
                        largest_component_nodes.erase(indexToV_map[index]);
                        frontier_nodes.erase(indexToV_map[index]);
                        boost::clear_vertex(indexToV_map[index],G);
                        // ROS_INFO("delete node %ld !!!", indexToV_map[index]);
                    }
                }
                    
            }
        }
        ROS_INFO("update graph");
    }

    std::pair<Vertex, double> robot_graphPose = get_robot_pose_in_graph(G,cur_pos,cur_quat,origin_x,origin_y,resolution,msg);
    history_map = msg->data;

    auto after_build_graph = std::chrono::high_resolution_clock::now();
    update_graph_time = after_build_graph - start;

    remove_non_largest_component(G);
    check_frontier(G,*msg);

    auto after_detect_frontier = std::chrono::high_resolution_clock::now();
    detect_frontier_time = after_detect_frontier - after_build_graph;

    cluster_frontier(G);

    auto after_cluster_frontier = std::chrono::high_resolution_clock::now();
    cluster_frontier_time = after_cluster_frontier - after_detect_frontier;

    
    if (!frontier_clusters.empty())
    {
        cost_yaw = low_level_network_planner(G,G_h,cur_pos,robot_graphPose.first,robot_graphPose.second,msg);
    }
    
    
   

    
    

    geometry_msgs::Twist vel_msg;


    
    
    
    auto g_ptr = std::make_shared<Graph>(G);
    VertexFilter vf(largest_component_nodes);
    EdgeFilter ef(g_ptr, largest_component_nodes);

    // 创建filtered_graph，（比较耗时，最好不用，用无序集合添加一个accessible的属性，后续规划路径时规划到accessible为false时跳过即可，
    //      但是可以用在可视化上，这样可视化简单且不计入探索算法的计算时间）
    subgraph fg(G, ef, vf);


    // add_allnodes_visualization(fg,marker_array);
    // add_alledges_visualization(fg,marker_array);
    add_LTNnodes_visualization(G,marker_array_LTN,true);
    // add_LTNnodes_visualization(G,marker_array_LTN,false);
    // add_allnodes_visualization(G,marker_array,true);
    add_alledges_visualization(G,marker_array,true);
    // add_allnodes_visualization(G_h,marker_array,false);
    // add_alledges_visualization(G_h,marker_array,false);
    add_path_visualization(G,path,marker_array);
    // add_room_visualization(G,rooms_vec,marker_array);
    add_frontier_visualization(G,marker_array);


    // std::size_t num_vertices_in_graph = boost::num_vertices(G);
    // std::size_t num_nodes_in_set = largest_component_nodes.size();
    // ROS_INFO("the origin vertexs number is %ld, filtered vertex is %ld",num_vertices_in_graph,num_nodes_in_set);

    // ROS_INFO("update map time is %d",update_map_times);
    update_map_times++;
    marker_pub.publish(marker_array);
    marker_pub_LTN.publish(marker_array_LTN);
    publishGoalMarker(goal_pub,history_pos);

    for (const Vertex& node : unaccessiable_nodes)
    {
        G[node].is_accessiable = true;
    }

    dwa_goal_yaw = std::atan2(dwa_goal_dir_y, dwa_goal_dir_x);
    dwa_goal_quaternion.setRPY(0, 0, dwa_goal_yaw);
    // 设置目标位置和朝向
    geometry_msgs::PoseStamped dwa_goal_posestamped;
    dwa_goal_posestamped.header.frame_id = "map"; // 确保使用正确的参考框架
    dwa_goal_posestamped.header.stamp = ros::Time::now();
    
    dwa_goal_posestamped.pose.position.x = dwa_goal_pos_x;
    dwa_goal_posestamped.pose.position.y = dwa_goal_pos_y;
    dwa_goal_posestamped.pose.position.z = 0; // 对于2D导航，Z通常是0
    
    dwa_goal_posestamped.pose.orientation = tf2::toMsg(dwa_goal_quaternion);
    
    // 发布目标
    dwa_goal_pub.publish(dwa_goal_posestamped);
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_dart");
    ros::NodeHandle nh("~");

    largest_component_nodes.max_load_factor(0.9);
    removed_from_largest_nodes.max_load_factor(0.9);
    frontier_nodes.max_load_factor(0.9);
    to_remove_frontier_nodes.max_load_factor(0.9);
    frontier_indices.max_load_factor(0.9);


    nh.param("sample_rate", sample_rate, 7);
    nh.param("rebuild_map_frequency", rebuild_map_frequency, 10);
    nh.param("check_safety_depth", check_safety_depth, 10);
    nh.param("frontier_threshold", frontier_threshold, 10);
    nh.param("cost_diff_threshold", cost_diff_threshold, 3.0);
    nh.param("velocity_step", velocity_step, 0.2);
    nh.param("turn_step", turn_step, 0.32);
    nh.param("occupied_grids_threshold", occupied_grids_threshold, 3);
    nh.param("cluster_info_threshold", cluster_info_threshold, 20);
    nh.param("roomToCluster_dis_threshold", roomToCluster_dis_threshold, 4.0);
    nh.param("roomToCluster_matchRate_threshold", roomToCluster_matchRate_threshold, 0.8);
    nh.param("erosion_round", erosion_round, 3);
    nh.param("safety_gain", safety_gain, 1.8);
    nh.param("goal_gain", goal_gain, 1.0);


    map_sub = nh.subscribe("/map", 10, mapCallback);

    // 初始化 Marker 发布器
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 5);
    marker_pub_LTN = nh.advertise<visualization_msgs::MarkerArray>("LTN", 5);
    goal_pub = nh.advertise<visualization_msgs::Marker>("target", 10);

    // 消融实验
    dwa_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    ros::spin();

    return 0;
}
