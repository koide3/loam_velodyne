#include <mutex>
#include <atomic>
#include <thread>
#include <iostream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include "loam_velodyne/LaserOdometry.h"
#include "loam_velodyne/LaserMapping.h"
#include "loam_velodyne/MultiScanRegistration.h"
#include "loam_velodyne/TransformMaintenance.h"

class LoamOffline {
public:
  LoamOffline()
  : nh(),
    scan_registration_nh("/multiScanRegistration"),
    laser_odometry_nh("/laserOdometry"),
    laser_mapping_nh("/laserMapping"),
    transform_maintenance_nh("/transformMaintenance"),
    multi_scan(),
    laser_odometry(laser_odometry_nh, 0.1),
    laser_mapping(laser_mapping_nh, 0.1),
    transform_maintenance()
  {
    if(!multi_scan.setup(nh, scan_registration_nh)) {
      ROS_ERROR("Failed to setup multi scan registration");
      exit(1);
    }

    if(!laser_odometry.setup(nh, laser_odometry_nh)) {
      ROS_ERROR("Failed to setup laser_odometry");
      exit(1);
    }

    if(!laser_mapping.setup(nh, laser_mapping_nh)) {
      ROS_ERROR("Failed to setup laser_mapping");
      exit(1);
    }

    if(!transform_maintenance.setup(nh, transform_maintenance_nh)) {
      ROS_ERROR("Failed to setup transform_maintenance");
      exit(1);
    }

    num_inputs = 0;
    num_outputs = 0;

    kill_switch = false;
    multi_scan_thread = std::thread([this]() { multi_scan_task(); });
    laser_odometry_thread = std::thread([this]() { laser_odometry_task(); });
    laser_mapping_thread = std::thread([this]() { laser_mapping_task(); });
    transform_maintenance_thread = std::thread([this]() { transform_maintenance_task(); });
  }

  ~LoamOffline() {
    kill_switch = true;
    multi_scan_thread.join();
    laser_odometry_thread.join();
    laser_mapping_thread.join();
    transform_maintenance_thread.join();
  }

  void print_status() {
    ROS_INFO_STREAM("inputs(" << num_inputs << ") => multi_scan(" << multi_scan_queue_size << ") => odometry(" << laser_odometry_queue_size << ") => mapping(" << laser_mapping_queue_size << ") => trans(" << transform_maintenance_queue_size << ") => outputs(" << num_outputs << ")" );
  }

  void join() {
    while(ros::ok() && num_inputs != num_outputs) {
      ros::WallDuration(0.1).sleep();
      print_status();

      std::lock_guard<std::mutex> lock(processed_queue_mutex);
      num_outputs += processed_queue.size();
      processed_queue.clear();
    }
  }

  void feed(sensor_msgs::PointCloud2::ConstPtr cloud_msg) {
    num_inputs ++;
    print_status();

    if(!ros::ok()) {
      kill_switch = true;
    }

    IOBoard::Ptr io_board = std::make_shared<IOBoard>();
    io_board->input_cloud = cloud_msg;

    while(true) {
      std::unique_lock<std::mutex> lock(multi_scan_queue_mutex);
      if(multi_scan_queue.size() > 100) {
        lock.unlock();
        ros::WallDuration(0.1).sleep();
        continue;
      }

      multi_scan_queue.push_back(io_board);
      break;
    }

    {
      std::lock_guard<std::mutex> lock(processed_queue_mutex);
      num_outputs += processed_queue.size();
      processed_queue.clear();
    }
  }

  void multi_scan_task() {
    while(!kill_switch && ros::ok()) {
      IOBoard::Ptr io_board;

      {
        std::unique_lock<std::mutex> lock(multi_scan_queue_mutex);
        if(multi_scan_queue.empty()) {
          lock.unlock();
          ros::WallDuration(0.1).sleep();
          continue;
        }

        io_board = multi_scan_queue.front();
        multi_scan_queue.pop_front();
        multi_scan_queue_size = multi_scan_queue.size();
      }

      multi_scan.handleCloudMessage(io_board);

      while(true) {
        std::unique_lock<std::mutex> lock(laser_odometry_queue_mutex);
        if(laser_odometry_queue.size() > 100) {
          lock.unlock();
          ros::WallDuration(0.1).sleep();
          continue;
        }

        laser_odometry_queue.push_back(io_board);
        break;
      }
    }
  }

  void laser_odometry_task() {
    while(!kill_switch && ros::ok()) {
      IOBoard::Ptr io_board;

      {
        std::unique_lock<std::mutex> lock(laser_odometry_queue_mutex);
        if(laser_odometry_queue.empty()) {
          lock.unlock();
          ros::WallDuration(0.1).sleep();
          continue;
        }

        io_board = laser_odometry_queue.front();
        laser_odometry_queue.pop_front();
        laser_odometry_queue_size = laser_odometry_queue.size();
      }

      laser_odometry.process(io_board);

      while(true) {
        std::unique_lock<std::mutex> lock(laser_mapping_queue_mutex);
        if(laser_mapping_queue.size() > 100) {
          lock.unlock();
          ros::WallDuration(0.1).sleep();
          continue;
        }

        laser_mapping_queue.push_back(io_board);
        break;
      }
    }
  }

  void laser_mapping_task() {
    while(!kill_switch && ros::ok()) {
      IOBoard::Ptr io_board;

      {
        std::unique_lock<std::mutex> lock(laser_mapping_queue_mutex);
        if(laser_mapping_queue.empty()) {
          lock.unlock();
          ros::WallDuration(0.1).sleep();
          continue;
        }

        io_board = laser_mapping_queue.front();
        laser_mapping_queue.pop_front();
        laser_mapping_queue_size = laser_mapping_queue.size();
      }

      laser_mapping.process(io_board);

      std::lock_guard<std::mutex> lock(transform_maintenance_queue_mutex);
      transform_maintenance_queue.push_back(io_board);
    }
  }

  void transform_maintenance_task() {
    while(!kill_switch && ros::ok()) {
      IOBoard::Ptr io_board;

      {
        std::unique_lock<std::mutex> lock(transform_maintenance_queue_mutex);
        if(transform_maintenance_queue.empty()) {
          lock.unlock();
          ros::WallDuration(0.1).sleep();
          continue;
        }

        io_board = transform_maintenance_queue.front();
        transform_maintenance_queue.pop_front();
        transform_maintenance_queue_size = transform_maintenance_queue.size();
      }

      transform_maintenance.process(io_board);

      std::lock_guard<std::mutex> lock(processed_queue_mutex);
      processed_queue.push_back(io_board);
    }
  }

private:
  ros::NodeHandle nh;
  ros::NodeHandle scan_registration_nh;
  ros::NodeHandle laser_odometry_nh;
  ros::NodeHandle laser_mapping_nh;
  ros::NodeHandle transform_maintenance_nh;

  loam::MultiScanRegistration multi_scan;
  loam::LaserOdometry laser_odometry;
  loam::LaserMapping laser_mapping;
  loam::TransformMaintenance transform_maintenance;

  std::atomic_bool kill_switch;
  std::thread multi_scan_thread;
  std::thread laser_odometry_thread;
  std::thread laser_mapping_thread;
  std::thread transform_maintenance_thread;

  std::deque<IOBoard::Ptr> multi_scan_queue;
  std::deque<IOBoard::Ptr> laser_odometry_queue;
  std::deque<IOBoard::Ptr> laser_mapping_queue;
  std::deque<IOBoard::Ptr> transform_maintenance_queue;
  std::deque<IOBoard::Ptr> processed_queue;

  std::mutex multi_scan_queue_mutex;
  std::mutex laser_odometry_queue_mutex;
  std::mutex laser_mapping_queue_mutex;
  std::mutex transform_maintenance_queue_mutex;
  std::mutex processed_queue_mutex;

  std::atomic_int num_inputs;
  std::atomic_int num_outputs;

  std::atomic_int multi_scan_queue_size;
  std::atomic_int laser_odometry_queue_size;
  std::atomic_int laser_mapping_queue_size;
  std::atomic_int transform_maintenance_queue_size;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "loam_offline");

  LoamOffline loam;

  rosbag::Bag bag("/home/koide/datasets/kitti/00.bag");
  if(!bag.isOpen()) {
    ROS_ERROR_STREAM("Failed to open " << bag.getFileName());
    return 1;
  }

  std::vector<std::string> topics = { "/velodyne_points" };
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ros::Time begin_time = view.getBeginTime();
  ros::Time end_time = view.getEndTime();
  std::deque<std::pair<ros::WallTime, ros::Time>> stamps;

  for(rosbag::MessageInstance const m : view) {
    stamps.push_back(std::make_pair(ros::WallTime::now(), m.getTime()));
    if(stamps.size() > 2) {
      const auto t0 = stamps.front();
      const auto t1 = stamps.back();

      double real_elapsed = (t1.first - t0.first).toSec();
      double sim_elapsed = (t1.second - t0.second).toSec();
      double play_rate = sim_elapsed / real_elapsed;

      double fraction = (m.getTime() - begin_time).toSec() / (end_time - begin_time).toSec();
      ROS_INFO_STREAM("progress: " << fraction << " " << play_rate << "X speeed");
    }

    if(stamps.size() > 15) {
      stamps.pop_front();
    }

    sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
    if(cloud_msg == nullptr) {
      continue;
    }

    loam.feed(cloud_msg);
  }

  loam.join();

  return 0;
}