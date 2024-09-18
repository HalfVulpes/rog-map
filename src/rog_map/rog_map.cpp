/**
* This file is part of ROG-Map
*
* Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/ROG-Map>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* ROG-Map is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ROG-Map is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
*/

#include "rog_map/rog_map.h"
#include <thread>
#include <chrono>

using namespace rog_map;

ROGMap::ROGMap(float resolution, float p_hit, float p_miss, float p_min, float p_max, float p_occ, float p_free){

    std::cout << "hihi" << endl;
    isInitialized = true;
    cfg_ = rog_map::Config();
    cfg_.resolution = resolution;
    cfg_.p_hit = p_hit;
    cfg_.p_miss = p_miss;
    cfg_.p_min = p_min;
    cfg_.p_max = p_max;
    cfg_.p_occ = p_occ;
    cfg_.p_free = p_free;
    cfg_.initialize();
    initProbMap();

    map_info_log_file_.open(DEBUG_FILE_DIR("rm_info_log.csv"), std::ios::out | std::ios::trunc);
    time_log_file_.open(DEBUG_FILE_DIR("rm_performance_log.csv"), std::ios::out | std::ios::trunc);

    robot_state_.p = cfg_.fix_map_origin;

    if (cfg_.map_sliding_en) {
        mapSliding(Vec3f(0, 0, 0));
        inf_map_->mapSliding(Vec3f(0, 0, 0));
    }
    else {
        /// if disable map sliding, fix map origin to (0,0,0)
        /// update the local map bound as
        local_map_bound_min_d_ = -cfg_.half_map_size_d + cfg_.fix_map_origin;
        local_map_bound_max_d_ = cfg_.half_map_size_d + cfg_.fix_map_origin;
        mapSliding(cfg_.fix_map_origin);
        inf_map_->mapSliding(cfg_.fix_map_origin);
    }

    /// Initialize visualization module
    // if (cfg_.visualization_en) {
    //     vm_.occ_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/occ", 1);
    //     vm_.unknown_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/unk", 1);
    //     vm_.occ_inf_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/inf_occ", 1);
    //     vm_.unknown_inf_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/inf_unk", 1);

    //     if (cfg_.frontier_extraction_en) {
    //         vm_.frontier_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/frontier", 1);
    //     }

    //     if (cfg_.esdf_en) {
    //         vm_.esdf_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/esdf", 1);
    //         vm_.esdf_neg_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/esdf/neg", 1);
    //         vm_.esdf_occ_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/esdf/occ", 1);
    //     }
    // }
    // vm_.mkr_arr_pub = nh_.advertise<visualization_msgs::MarkerArray>("rog_map/map_bound", 1);

    // if (cfg_.ros_callback_en) {
    //     rc_.odom_sub = nh_.subscribe(cfg_.odom_topic, 1, &ROGMap::odomCallback, this);
    //     rc_.cloud_sub = nh_.subscribe(cfg_.cloud_topic, 1, &ROGMap::cloudCallback, this);
    //     rc_.update_timer = nh_.createTimer(ros::Duration(0.001), &ROGMap::updateCallback, this);
    // }

    writeMapInfoToLog(map_info_log_file_);
    map_info_log_file_.close();
    // for (int i = 0; i < time_consuming_name_.size(); i++) {
    //     time_log_file_ << time_consuming_name_[i];
    //     if (i != time_consuming_name_.size() - 1) {
    //         time_log_file_ << ", ";
    //     }
    // }
    //time_log_file_ << endl;


    // if (cfg_.load_pcd_en) {
    //     string pcd_path = cfg_.pcd_name;
    //     PointCloud::Ptr pcd_map(new PointCloud);
    //     if (pcl::io::loadPCDFile(pcd_path, *pcd_map) == -1) {
    //         cout << RED << "Load pcd file failed!" << RESET << endl;
    //         exit(-1);
    //     }
    //     Pose cur_pose;
    //     cur_pose.first = Vec3f(0, 0, 0);
    //     updateOccPointCloud(*pcd_map);
    //     esdf_map_->updateESDF3D(robot_state_.p);
    //     cout << BLUE << " -- [ROGMap]Load pcd file success with " << pcd_map->size() << " pts." << RESET << endl;
    //     map_empty_ = false;
    // }
}

bool ROGMap::isLineFree(const rog_map::Vec3f& start_pt, const rog_map::Vec3f& end_pt,
                        const bool& use_inf_map, const bool& use_unk_as_occ) const {
    if(start_pt.array().isNaN().any() || end_pt.array().isNaN().any() ) {
        cout<<RED<<" -- [ROGMap] Call isLineFree with NaN in start or end pt, return false."<<RESET<<endl;
        return false;
    }
    raycaster::RayCaster raycaster;
    if (use_inf_map) {
        raycaster.setResolution(cfg_.inflation_resolution);
    }
    else {
        raycaster.setResolution(cfg_.resolution);
    }
    Vec3f ray_pt;
    raycaster.setInput(start_pt, end_pt);
    while (raycaster.step(ray_pt)) {
        if (!use_unk_as_occ) {
            // allow both unk and free
            if (use_inf_map) {
                if (isOccupiedInflate(ray_pt)) {
                    return false;
                }
            }
            else {
                if (isOccupied(ray_pt)) {
                    return false;
                }
            }
        }
        else {
            // only allow known free
            if (use_inf_map) {
                if ((isUnknownInflate(ray_pt) || isOccupiedInflate(ray_pt)))
                    return false;
            }
            else {
                if (!isKnownFree(ray_pt)) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool ROGMap::isLineFree(const Vec3f& start_pt, const Vec3f& end_pt, const double& max_dis,
                        const vec_Vec3i& neighbor_list) const {
    raycaster::RayCaster raycaster;
    raycaster.setResolution(cfg_.resolution);
    Vec3f ray_pt;
    raycaster.setInput(start_pt, end_pt);
    while (raycaster.step(ray_pt)) {
        if (max_dis > 0 && (ray_pt - start_pt).norm() > max_dis) {
            return false;
        }

        if (neighbor_list.empty()) {
            if (isOccupied(ray_pt)) {
                return false;
            }
        }
        else {
            Vec3i ray_pt_id_g;
            posToGlobalIndex(ray_pt, ray_pt_id_g);
            for (const auto& nei : neighbor_list) {
                Vec3i shift_tmp = ray_pt_id_g + nei;
                if (isOccupied(shift_tmp)) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool ROGMap::isLineFree(const Vec3f& start_pt, const Vec3f& end_pt, Vec3f& free_local_goal, const double& max_dis,
                        const vec_Vec3i& neighbor_list) const {
    raycaster::RayCaster raycaster;
    raycaster.setResolution(cfg_.resolution);
    Vec3f ray_pt;
    raycaster.setInput(start_pt, end_pt);
    free_local_goal = start_pt;
    while (raycaster.step(ray_pt)) {
        free_local_goal = ray_pt;
        if (max_dis > 0 && (ray_pt - start_pt).norm() > max_dis) {
            return false;
        }

        if (neighbor_list.empty()) {
            if (isOccupied(ray_pt)) {
                return false;
            }
        }
        else {
            Vec3i ray_pt_id_g;
            posToGlobalIndex(ray_pt, ray_pt_id_g);
            for (const auto& nei : neighbor_list) {
                Vec3i shift_tmp = ray_pt_id_g + nei;
                if (isOccupied(shift_tmp)) {
                    return false;
                }
            }
        }
    }
    free_local_goal = end_pt;
    return true;
}

void ROGMap::updateMap(const PointCloud& cloud, const Pose& pose) {
    if (cfg_.ros_callback_en) {
        std::cout << RED << "ROS callback is enabled, can not insert map from updateMap API." << RESET
            << std::endl;
        return;
    }

    if (cloud.empty()) {
        static int local_cnt = 0;
        if (local_cnt++ > 100) {
            cout << YELLOW << "No cloud input, please check the input topic." << RESET << endl;
            local_cnt = 0;
        }
        return;
    }
    std::cout << "ProbMap update" << endl;
    updateProbMap(cloud, pose);
}

RobotState ROGMap::getRobotState() const {
    return robot_state_;
}

void ROGMap::updateRobotState(const Pose& pose) {
    robot_state_.p = pose.first;
    robot_state_.q = pose.second;
    robot_state_.yaw = get_yaw_from_quaternion<double>(pose.second);
    updateLocalBox(pose.first);
}


void ROGMap::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
    updateRobotState(std::make_pair(
        Vec3f(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
              odom_msg->pose.pose.position.z),
        Quatf(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
              odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z)));


    static tf2_ros::TransformBroadcaster br_map_ego;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "drone";
    transformStamped.transform.translation.x = odom_msg->pose.pose.position.x;
    transformStamped.transform.translation.y = odom_msg->pose.pose.position.y;
    transformStamped.transform.translation.z = odom_msg->pose.pose.position.z;
    transformStamped.transform.rotation.x = odom_msg->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odom_msg->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odom_msg->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = odom_msg->pose.pose.orientation.w;
    br_map_ego.sendTransform(transformStamped);
}

void ROGMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    if (!robot_state_.rcv) {
        return;
    }
    double cbk_t = ros::Time::now().toSec();
    if (cbk_t - robot_state_.rcv_time > cfg_.odom_timeout) {
        std::cout << YELLOW << " -- [ROS] Odom timeout, skip cloud callback." << RESET << std::endl;
        return;
    }
    pcl::fromROSMsg(*cloud_msg, rc_.pc);
    rc_.updete_lock.lock();
    rc_.pc_pose = std::make_pair(robot_state_.p, robot_state_.q);
    rc_.unfinished_frame_cnt++;
    map_empty_ = false;
    rc_.updete_lock.unlock();
}

void ROGMap::updateCallback(const ros::TimerEvent& event) {
    if (map_empty_) {
        static double last_print_t = ros::Time::now().toSec();
        double cur_t = ros::Time::now().toSec();
        if (cfg_.ros_callback_en && (cur_t - last_print_t > 1.0)) {
            std::cout << YELLOW << " -- [ROG WARN] No point cloud input, check the topic name." << RESET << std::endl;
            last_print_t = cur_t;
        }
        return;
    }
    if (rc_.unfinished_frame_cnt == 0) {
        return;
    }
    else if (rc_.unfinished_frame_cnt > 1) {
        std::cout << RED <<
            " -- [ROG WARN] Unfinished frame cnt > 1, the map may not work in real-time" << RESET
            << std::endl;
    }
    static PointCloud temp_pc;
    static Pose temp_pose;
    rc_.updete_lock.lock();
    temp_pc = rc_.pc;
    temp_pose = rc_.pc_pose;
    rc_.unfinished_frame_cnt = 0;
    rc_.updete_lock.unlock();

    updateProbMap(temp_pc, temp_pose);

    writeTimeConsumingToLog(time_log_file_);
}

void ROGMap::vecEVec3fToPC2(const vec_E<Vec3f>& points, sensor_msgs::PointCloud2& cloud) {
    // 设置header信息
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.resize(points.size());
    for (long unsigned int i = 0; i < points.size(); i++) {
        pcl_cloud[i].x = static_cast<float>(points[i][0]);
        pcl_cloud[i].y = static_cast<float>(points[i][1]);
        pcl_cloud[i].z = static_cast<float>(points[i][2]);
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "world";
}

float ROGMap::getOcc(){
    return cfg_.l_occ;

}

float ROGMap::getFree(){
    return cfg_.l_free;
}

Pose parsePose(const std::string& poseStr) {
    std::istringstream ss(poseStr);
    std::string token;
    float values[7];
    int idx = 0;

    // Parse the string and extract floats
    while (std::getline(ss, token, ',') && idx < 7) {
        values[idx++] = std::stof(token);
    }

    // Construct position vector
    Vec3f position(values[0], values[1], values[2]);

    // Construct quaternion (note: Eigen uses (w, x, y, z) ordering)
    Eigen::Quaterniond orientation(values[6], values[3], values[4], values[5]);

    return Pose(position, orientation);
}


std::vector<Pose> loadPoses(const std::string& filename) {
    std::vector<Pose> poses;
    std::ifstream file(filename);
    std::string line;

    // Skip the header line
    getline(file, line);

    while (getline(file, line)) {
        std::istringstream iss(line);
        Pose pose;

        // Skip the sequence number
        std::string tmp;
        getline(iss, tmp, ',');

        // Parse the translation part (tx, ty, tz)
        float tx, ty, tz;
        getline(iss, tmp, ',');
        tx = std::stof(tmp);
        getline(iss, tmp, ',');
        ty = std::stof(tmp);
        getline(iss, tmp, ',');
        tz = std::stof(tmp);

        // Set the Vec3f part (translation)
        pose.first = Vec3f(tx, ty, tz);

        // Parse the quaternion part (qx, qy, qz, qw)
        double qx, qy, qz, qw;
        getline(iss, tmp, ',');
        qx = std::stod(tmp);
        getline(iss, tmp, ',');
        qy = std::stod(tmp);
        getline(iss, tmp, ',');
        qz = std::stod(tmp);
        getline(iss, tmp, ',');
        qw = std::stod(tmp);

        // Set the Eigen::Quaterniond part (rotation)
        pose.second = Eigen::Quaterniond(qw, qx, qy, qz);

        // Add the parsed Pose to the vector
        poses.push_back(pose);
    }

    file.close();
    return poses;
}


void loadPointCloud(const std::string& filename, std::vector<std::pair<PointCloud::Ptr, Pose>>& clouds_and_poses, int index, const std::vector<Pose>& poses) {
    PointCloud::Ptr cloud(new PointCloud);
    if (pcl::io::loadPLYFile<PclPoint>(filename, *cloud) == -1) {
        std::cerr << "Failed to load " << filename << std::endl;
        return;
    }

    if (index < poses.size()) {
        clouds_and_poses.push_back(std::make_pair(cloud, poses[index]));
    } else {
        std::cerr << "Pose data not available for " << filename << std::endl;
    }
}


int main(int argc, char** argv){

    std::cout << "launched" << endl;

    // Variables to store command-line arguments
    std::string input_path;
    int start = 0;
    int end = 0;
    std::string pose_file;
    float resolution = 0.1f;
    float p_hit = 0.60f;
    float p_miss = 0.40f;
    float p_min = 0.12f;
    float p_max = 0.97f;
    float p_occ = 0.60f;
    float p_free = 0.40f;

    // Parse command-line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--input_path" && i + 1 < argc) {
            input_path = argv[++i];
        } else if (arg == "--start" && i + 1 < argc) {
            start = std::stoi(argv[++i]);
        } else if (arg == "--end" && i + 1 < argc) {
            end = std::stoi(argv[++i]);
        } else if (arg == "--poses" && i + 1 < argc) {
            pose_file = argv[++i];
        } else if (arg == "--resolution" && i + 1 < argc) {
            resolution = std::stof(argv[++i]);
        } else if (arg == "--p_hit" && i + 1 < argc) {
            p_hit = std::stof(argv[++i]);
        } else if (arg == "--p_miss" && i + 1 < argc) {
            p_miss = std::stof(argv[++i]);
        } else if (arg == "--p_min" && i + 1 < argc) {
            p_min = std::stof(argv[++i]);
        } else if (arg == "--p_max" && i + 1 < argc) {
            p_max = std::stof(argv[++i]);
        } else if (arg == "--p_occ" && i + 1 < argc) {
            p_occ = std::stof(argv[++i]);
        } else if (arg == "--p_free" && i + 1 < argc) {
            p_free = std::stof(argv[++i]);
        } else {
            std::cerr << "Unknown or incomplete argument: " << arg << std::endl;
            std::cerr << "Usage: ./rog_map --input_path <path> --start <start> --end <end> --poses <pose_file> --resolution <resolution> --p_hit <p_hit> --p_miss <p_miss> --p_min <p_min> --p_max <p_max> --p_occ <p_occ> --p_free <p_free>" << std::endl;
            return 1;
        }
    }

    // Validate arguments
    if (input_path.empty() || pose_file.empty() || start >= end) {
        std::cerr << "Invalid arguments." << std::endl;
        std::cerr << "Usage: ./rog_map --input_path <path> --start <start> --end <end> --poses <pose_file>" << std::endl;
        return 1;
    }

    // Ensure input_path ends with '/'
    if (input_path.back() != '/' && input_path.back() != '\\') {
        input_path += '/';
    }



    // Create a ROS node handle

    ROGMap* map = new ROGMap(resolution, p_hit, p_miss, p_min, p_max, p_occ, p_free);
    
    if (map->isInitialized) {
        std::cout << "Map initialized successfully." << std::endl;
    }

    std::vector<Pose> poses = loadPoses(pose_file);
    std::vector<std::pair<PointCloud::Ptr, Pose>> clouds_and_poses;

    // Load all point clouds
    std::vector<std::thread> loaders;
    for (int i = start; i < end; ++i) {
        std::stringstream ss;
        ss << std::setw(6) << std::setfill('0') << i;
        std::string index_str = ss.str();
        std::string inputFilename = input_path + "point_cloud_" + index_str + ".ply";

        loaders.emplace_back(loadPointCloud, inputFilename, std::ref(clouds_and_poses), i, std::ref(poses));
        if (loaders.size() >= std::thread::hardware_concurrency()) {
            for (auto& th : loaders) th.join();
            loaders.clear();
        }
    }
    for (auto& th : loaders) th.join();

    map->initVisualization();

    int curr = start;

    for (auto& cloud_pose : clouds_and_poses) {
        std::cout << "Currently updating " << curr << endl;
        curr++;
        map->updateMap(*cloud_pose.first, cloud_pose.second);

        // Sleep for the desired duration, processing visualization events
        int total_sleep_ms = 300; // Total sleep duration in milliseconds
        int sleep_interval_ms = 10; // Sleep interval in milliseconds

        int elapsed_time = 0;
        while (elapsed_time < total_sleep_ms) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_interval_ms));
            elapsed_time += sleep_interval_ms;

            // Process visualization events
            map->viewer->spinOnce(1);
        }
    }

    std::cout << "Processing completed." << std::endl;
    return 0;
}
