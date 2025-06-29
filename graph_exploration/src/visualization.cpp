#include "function.h"

void add_LTNnodes_visualization(Graph& g, visualization_msgs::MarkerArray& marker_array, bool is_periphery){
    // subgraph_vertex_iter vi, vend;
    vertex_iter vi, vend;
    for (boost::tie(vi, vend) = vertices(g); vi != vend; ++vi) {
        Vertex v = *vi;
        // double adjusted_x = g[v].pos.first + origin_x;
        // double adjusted_y = g[v].pos.second + origin_y;
        // Create and publish the marker for visualization
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.ns = is_periphery ? "periphery" : "frontier";
        marker.header.stamp = ros::Time::now();
        marker.id = v; // Here using the vertex_descriptor as ID
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = g[v].pos.first;
        marker.pose.position.y = g[v].pos.second;
        
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.1;
        // if (!is_periphery && g[v].frontier_size > frontier_threshold) {  // 或其他你的逻辑条件
        // // if (degree(v,g)<8){
        //     marker.color.r = 0.8;
        //     marker.color.g = 0.1;
        //     marker.color.b = 0.1;
        //     marker.color.a = 1.0; // 确保可见性
        //     marker.pose.position.z = 0.1;
        //     marker_array.markers.push_back(marker);
        // }
        if (is_periphery)
        {
            // if (degree(v,g)<8)
            if (g[v].is_accessiable)
            {
                marker.color.r = 0.0;
                marker.color.g = 0.6;
                marker.color.b = 0.4;
                marker.color.a = 1.0;
                marker.pose.position.z = 0.0;
                marker_array.markers.push_back(marker);
            }
            else
            {
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                marker.color.a = 1.0; // 确保可见性
                marker.pose.position.z = 0.0;
                marker_array.markers.push_back(marker);
            } 
        }

        // Create and publish the text marker for displaying the Vertex value
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = is_periphery ? "periphery" : "frontier";
        text_marker.id = v + 100000; // Ensure a unique ID distinct from sphere markers
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = g[v].pos.first;
        text_marker.pose.position.y = g[v].pos.second;
        text_marker.pose.position.z = 0.2; // Slightly above the node marker
        if (!is_periphery && g[v].frontier_size > frontier_threshold)
        {
            text_marker.pose.position.z = 0.2; // Slightly above the node marker
            text_marker.scale.z = 0.25; // Text size
            text_marker.color.a = 1.0;
            text_marker.color.r = 0.0;
            text_marker.color.g = 0.0;
            text_marker.color.b = 0.0;
            text_marker.text = std::to_string(g[v].frontier_size);
            marker_array.markers.push_back(text_marker);
        }
        else if (is_periphery)
        {
            text_marker.pose.position.z = 0.05; // Slightly above the node marker
            text_marker.scale.z = 0.25; // Text size
            text_marker.color.a = 1.0;
            text_marker.color.r = 0.9;
            text_marker.color.g = 0.9;
            text_marker.color.b = 0.9;
            text_marker.text = std::to_string(degree(v,g));
            marker_array.markers.push_back(text_marker);
        }
        
        // int edge_number = degree(v,g);
        // text_marker.text = std::to_string(g[v].frontier_size);
        // text_marker.text = std::to_string(g[v].frontier_size);
        // text_marker.text = std::to_string(edge_number);

        // marker_array.markers.push_back(text_marker);
    }
    
}



void add_alledges_visualization(Graph& g, visualization_msgs::MarkerArray& marker_array, bool is_low_level_network) {
    // subgraph_edge_iter it, it_end;
    edge_iter it, it_end;
    

    // auto edge_pair = edges(g);
    edge_id_counter = 0;
    // for (auto it = edge_pair.first; it != edge_pair.second; ++it)
    for (boost::tie(it, it_end) = edges(g); it != it_end; ++it) {
        Vertex u = source(*it, g);  // 获取边的起点
        Vertex v = target(*it, g);  // 获取边的终点
        // std::cout << "Edge connects vertex " << u << " and vertex " << v << std::endl;
    
        auto& p_u = g[u].pos; // 起点坐标
        auto& p_v = g[v].pos; // 终点坐标

        
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = is_low_level_network ? "edges" : "high_level_edges";
        // line_marker.id = std::hash<Vertex>{}(u) ^ std::hash<Vertex>{}(v); // Generate a unique ID based on vertex descriptors
        line_marker.id = edge_id_counter;
        line_marker.type = visualization_msgs::Marker::LINE_LIST;
        line_marker.action = visualization_msgs::Marker::ADD;
        if (is_low_level_network)
        {
            line_marker.scale.x = 0.05; // 线条的粗细
            line_marker.color.a = 1.0;
            line_marker.color.r = 0.0;
            line_marker.color.g = 0.0;
            line_marker.color.b = 0.0;
        }
        else
        {
            line_marker.scale.x = 0.07; // 线条的粗细
            line_marker.color.a = 1.0;
            line_marker.color.r = 0.5;
            line_marker.color.g = 0.5;
            line_marker.color.b = 0.5;
        }
        
        
        // Initialize the quaternion
        line_marker.pose.orientation.x = 0.0;
        line_marker.pose.orientation.y = 0.0;
        line_marker.pose.orientation.z = 0.0;
        line_marker.pose.orientation.w = 1.0;
        // 设置线的起点和终点
        geometry_msgs::Point start;
        start.x = p_u.first;
        start.y = p_u.second;
        start.z = 0;
        geometry_msgs::Point end;
        end.x = p_v.first;
        end.y = p_v.second;
        end.z = 0;

        line_marker.points.push_back(start);
        line_marker.points.push_back(end);

        marker_array.markers.push_back(line_marker);
        edge_id_counter++;
        // 确认点已经被加入到marker_array
        // ROS_INFO("Edge from (%f, %f) to (%f, %f) added to marker array",
        //          pos[u].first, pos[u].second, pos[v].first, pos[v].second);
    }
}

void add_path_visualization(Graph& g, std::vector<Vertex> path, visualization_msgs::MarkerArray& marker_array){
    int index = 0;
    for (Vertex& path_node: path)
    {
        // 初始化marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";  // 设置frame
        marker.header.stamp = ros::Time::now();
        marker.ns = "path";
        marker.id = index;
        marker.type = visualization_msgs::Marker::CYLINDER;  // 使用圆柱体表示圆形
        marker.action = visualization_msgs::Marker::ADD;

        // 设置圆形的位置和大小
        marker.pose.position.x = g[path_node].pos.first;
        marker.pose.position.y = g[path_node].pos.second;
        marker.pose.position.z = 0.2;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.3;  // 直径为1
        marker.scale.y = 0.3;  // 直径为1
        marker.scale.z = 0.01; // 非常薄的圆柱体，看起来像一个圆

        // 设置颜色和透明度
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;  // 完全不透明
        index++;
        marker_array.markers.push_back(marker);
    }
    
}



void add_frontier_visualization(Graph& g, visualization_msgs::MarkerArray& marker_array){
    int index = 0;
    for (std::pair<Vertex,Vertex_set>& cluster: frontier_clusters)
    {
        int information_value = 0;
        for (Vertex v: cluster.second)
        {
            information_value += g[v].frontier_size;
            if (information_value > 40)
            {
                break;
            }
        }
        
        // 初始化marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";  // 设置frame
        marker.header.stamp = ros::Time::now();
        marker.ns = "frontier_cluster";
        marker.id = index;
        marker.type = visualization_msgs::Marker::CYLINDER;  // 使用圆柱体表示圆形
        marker.action = visualization_msgs::Marker::ADD;

        // 设置圆形的位置和大小
        marker.pose.position.x = g[cluster.first].pos.first;
        marker.pose.position.y = g[cluster.first].pos.second;
        marker.pose.position.z = 0.2;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        // marker.scale.x = 0.2;  
        // marker.scale.y = 0.2;  
        marker.scale.x = 0.2 * information_value / 40.0;  
        marker.scale.y = 0.2 * information_value / 40.0;  
        marker.scale.z = 0.3; // 非常薄的圆柱体，看起来像一个圆

        if(unaccessiable_nodes.find(cluster.first) == unaccessiable_nodes.end())
        {   marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;  // 完全不透明
        }
        else
        {
            marker.color.r = 143 / 255.0;
            marker.color.g = 188 / 255.0;
            marker.color.b = 143 / 255.0;
            marker.color.a = 1.0; // 不透明
        }
        

        index++;
        marker_array.markers.push_back(marker);
    }
}

void publishGoalMarker(const ros::Publisher& marker_pub, std::pair<double, double>& target_pose) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";  // 使用与地图相同的坐标系
    marker.header.stamp = ros::Time::now();  // 设置时间戳
    marker.ns = "goal_marker";  // 设置namespace
    marker.id = 0;  // 每个marker需要一个唯一的ID
    marker.type = visualization_msgs::Marker::SPHERE;  // 设置类型为球形
    marker.action = visualization_msgs::Marker::ADD;  // 添加或更新marker

    // 设置位置
    marker.pose.position.x = target_pose.first;
    marker.pose.position.y = target_pose.second;
    marker.pose.position.z = 0.3;

    // 设置方向（无旋转）
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // 设置尺寸（例如，直径为0.5米的球）
    marker.scale.x = 0.4;  // 直径
    marker.scale.y = 0.4;
    marker.scale.z = 0.1;

    // 设置颜色和透明度
    if (history_network)
    {
        marker.color.r = 0.8;  // 红色
        marker.color.g = 0.2;
        marker.color.b = 0.0;
        marker.color.a = 1.0;  // 不透明
    }
    else
    {
        marker.color.r = 0.2;  // 红色
        marker.color.g = 0.8;
        marker.color.b = 0.0;
        marker.color.a = 1.0;  // 不透明
    }
    
    
    

    // 发布marker
    marker_pub.publish(marker);
}