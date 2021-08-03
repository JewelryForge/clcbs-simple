#include "CarModel.h"
#include "LocalPlanner.h"
#include <iostream>
#include "ctime"
#include "GlobalPlanner.h"
#include <boost/program_options.hpp>
#include "PlanVisualizer.h"
#include <xmlrpcpp/XmlRpcValue.h>
#include <io.hpp>
using namespace std;

double toNum(const XmlRpc::XmlRpcValue &v) {
  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) return int(v);
  else return double(v);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Tracking");
  ros::NodeHandle nh;
  bool is_sim, is_real;
  std::string which_agent;
  XmlRpc::XmlRpcValue schedule;
  double time_speed = 1.0;
  nh.getParam("is_sim", is_sim);
  nh.getParam("is_real", is_real);
  nh.getParam("agent_binding", which_agent);
  nh.getParam("/config/schedule", schedule);
  nh.param("time_speed", time_speed, 1.0);
  Constants::loadFromRosParam();

  std::vector<std::unique_ptr<LocalPlannerBase>> controllers;
  PlanVisualizer visualizer;
  int count = 0;
  for (const auto &a : schedule) {
    std::string agent = a.first;
    ROS_INFO_STREAM("SET UP " << agent);
    std::vector<std::pair<double, State>> t_states;
    for (int i = 0; i < a.second.size(); i++) {
      const auto &mid_point = a.second[i];
      t_states.emplace_back(toNum(mid_point["t"]) / time_speed,
                            State(toNum(mid_point["x"]) - Constants::MAP_SIZE_X / 2,
                                  toNum(mid_point["y"]) - Constants::MAP_SIZE_Y / 2,
                                  toNum(mid_point["yaw"])));
    }
    visualizer.addPlan(t_states);
    controllers.push_back(std::make_unique<LocalPlannerSim>(agent, t_states));
    // if (count++ != 3) controllers.push_back(std::make_unique<LocalPlannerSim>(agent, t_states));
    // else controllers.push_back(std::make_unique<LocalPlannerSim>("champ", t_states));
//    if (is_real && agent == which_agent) controllers.push_back(std::make_unique<LocalPlannerSL>(agent, t_states));
  }

  ROS_INFO("SETTING UP FINISHED");
  ros::Duration(0.5).sleep();

  // Publish path to RVIZ
  if (is_sim) {
    visualizer.publishOnce();
    ros::Duration(0.5).sleep();
  }

  // Start controlling
  ros::spinOnce();
  while (ros::ok() && !LocalPlannerBase::activateAll()){
    ros::Duration(1).sleep();
    ros::spinOnce();
  };
  auto rate = ros::Rate(50);
  while (ros::ok()) {
    ros::spinOnce();
    for (auto &ptr : controllers) ptr->calculateVelocityAndPublish();
    rate.sleep();
  }
}