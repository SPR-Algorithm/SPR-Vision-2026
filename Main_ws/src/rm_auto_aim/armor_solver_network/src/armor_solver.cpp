// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "armor_solver/armor_solver.hpp"
// std
#include <cmath>
#include <cstddef>
#include <stdexcept>
// project
#include "armor_solver/armor_solver_node.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"

namespace fyt::auto_aim {
Solver::Solver(std::weak_ptr<rclcpp::Node> n) : node_(n) {
  auto node = node_.lock();

  shooting_range_w_ = node->declare_parameter("solver.shooting_range_width", 0.3);
  shooting_range_h_ = node->declare_parameter("solver.shooting_range_height", 0.3);
  max_tracking_v_yaw_ = node->declare_parameter("solver.max_tracking_v_yaw", 6.0);
  prediction_delay_ = node->declare_parameter("solver.prediction_delay", -0.1);
  controller_delay_ = node->declare_parameter("solver.controller_delay", 0.0);
  side_angle_ = node->declare_parameter("solver.side_angle", 15.0);
  min_switching_v_yaw_ = node->declare_parameter("solver.min_switching_v_yaw", 1.0);

  std::string compenstator_type = node->declare_parameter("solver.compensator_type", "ideal");
  trajectory_compensator_ = CompensatorFactory::createCompensator(compenstator_type);
  trajectory_compensator_->iteration_times = node->declare_parameter("solver.iteration_times", 20);
  trajectory_compensator_->velocity = node->declare_parameter("solver.bullet_speed", 27.0);
  trajectory_compensator_->gravity = node->declare_parameter("solver.gravity", 9.8);
  trajectory_compensator_->resistance = node->declare_parameter("solver.resistance", 0.001);

  manual_compensator_ = std::make_unique<ManualCompensator>();
  auto angle_offset = node->declare_parameter("solver.angle_offset", std::vector<std::string>{});
  if(!manual_compensator_->updateMapFlow(angle_offset)) {
    FYT_WARN("armor_solver", "Manual compensator update failed!");
  }

  state = State::TRACKING_ARMOR;
  overflow_count_ = 0;
  transfer_thresh_ = 5;

  node.reset();
}

rm_interfaces::msg::GimbalCmd Solver::solve(const rm_interfaces::msg::Target &target,
                                            const rclcpp::Time &current_time,
                                            std::shared_ptr<tf2_ros::Buffer> tf2_buffer_) {
  // Get newest parameters 
  try {
    auto node = node_.lock();
    max_tracking_v_yaw_ = node->get_parameter("solver.max_tracking_v_yaw").as_double();
    prediction_delay_ = node->get_parameter("solver.prediction_delay").as_double();
    controller_delay_ = node->get_parameter("solver.controller_delay").as_double();
    side_angle_ = node->get_parameter("solver.side_angle").as_double();
    min_switching_v_yaw_ = node->get_parameter("solver.min_switching_v_yaw").as_double();
    node.reset();
  } catch (const std::runtime_error &e) {
    FYT_ERROR("armor_solver", "{}", e.what());
  }

  // Get current roll, yaw and pitch of gimbal
  try {
    auto gimbal_tf =
      tf2_buffer_->lookupTransform(target.header.frame_id, "gimbal_link", tf2::TimePointZero);
    auto msg_q = gimbal_tf.transform.rotation;

    tf2::Quaternion tf_q;
    tf2::fromMsg(msg_q, tf_q);
    tf2::Matrix3x3(tf_q).getRPY(rpy_[0], rpy_[1], rpy_[2]);
    rpy_[1] = -rpy_[1];
  } catch (tf2::TransformException &ex) {
    FYT_ERROR("armor_solver", "{}", ex.what());
    throw ex;
  }


  // Use flying time to approximately predict the position of target
  Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);
  double target_yaw = target.yaw;
  double flying_time = trajectory_compensator_->getFlyingTime(target_position);
  double dt =
    (current_time - rclcpp::Time(target.header.stamp)).seconds() + flying_time + prediction_delay_;

  // //fix flyingtime 
  // int iterrations =3;
  // for(int i=0;i<iterrations;i++){
  //   //std::cout<<"迭代次数: "<<i<<std::endl;
  //   target_position.x() += dt * target.velocity.x;
  //   target_position.y() += dt * target.velocity.y;
  //   target_position.z() += dt * target.velocity.z;
  //   target_yaw += dt * target.v_yaw;

  //   std::vector<ArmorData> current_armors;

  //   if(target.armors_num ==4){//r1是当前装甲板的轴向半径，
  //     // printf("armors_num %d",target_data.armors_num);
  //     // printf("\n");
  
  //     //计算四块装甲板的位置
  //     //target发布的
  //     ArmorData armor1;
  //     armor1.x = target_position.x() - target.radius_1 * cos(target_yaw);
  //     armor1.y = target_position.y() - target.radius_1 * sin(target_yaw);
  //     armor1.z = target_position.z() + target.d_zc;
  //     armor1.yaw = target_yaw;
  
  //     //target顺时针旋转90度的
  //     ArmorData armor2;
  //     armor2.x = target_position.x() - target.radius_2 * cos(target_yaw+M_PI/2);
  //     armor2.y = target_position.y() - target.radius_2 * sin(target_yaw+M_PI/2);
  //     armor2.z = target_position.z() + target.d_zc + target.d_za;//当前装甲是较低，dz>0，较高,dz<0
  //     armor2.yaw = target_yaw+M_PI/2;
  
  //     //target逆时针旋转90度的
  //     ArmorData armor3;
  //     armor3.x = target_position.x() - target.radius_2 * cos(target_yaw-M_PI/2);
  //     armor3.y = target_position.y() - target.radius_2 * sin(target_yaw-M_PI/2);
  //     armor3.z = target_position.z() + target.d_zc + target.d_za;//当前装甲是较低的，dz>0，较高的dz<0
  //     armor3.yaw = target_yaw-M_PI/2;
  
  
  //     //target顺时针旋转180度的
  //     ArmorData armor4;
  //     armor4.x = target_position.x() - target.radius_1 * cos(target_yaw+M_PI);
  //     armor4.y = target_position.y() - target.radius_1 * sin(target_yaw+M_PI);
  //     armor4.z = target_position.z() + target.d_zc;
  //     armor4.yaw = target_yaw+M_PI;
  
  //     //将四块装甲板的数据存入vector
  //     current_armors.push_back(armor1);
  //     current_armors.push_back(armor2);
  //     current_armors.push_back(armor3);
  //     current_armors.push_back(armor4);
  
  
  //   }
  
  //   else if (target.armors_num == 3)
  //   {
  //     // printf("armors_num %d",target_data.armors_num);
  //     // printf("\n");
  
  //     //printf("预测yaw预量: %f \n",float(yaw-target_data.yaw));
  //     //计算三块装甲板的位置
  //     //target发布的
  //     ArmorData armor1;
  //     armor1.x = target_position.x() - target.radius_1 * cos(target_yaw);
  //     armor1.y = target_position.y() - target.radius_1 * sin(target_yaw);
  //     armor1.z = target_position.z() + target.d_zc;
  //     armor1.yaw = target_yaw;
  //     //printf("armor1_yaw %f \n",armor1.yaw);
  
  //     //target顺时针旋转120度的
  //     ArmorData armor2;
  //     armor2.x = target_position.x() - target.radius_1 * cos(target_yaw+M_PI*2/3);
  //     armor2.y = target_position.y() - target.radius_1 * sin(target_yaw+M_PI*2/3);
  //     armor2.z = target_position.z() + target.d_zc;//当前装甲是较低的，dz>0，较高的dz<0
  //     armor2.yaw = target_yaw+M_PI*2/3;
  //     //printf("armor2_yaw %f \n",armor2.yaw);
  
  //     //target逆时针旋转120度的
  //     ArmorData armor3;
  //     armor3.x = target_position.x() - target.radius_1 * cos(target_yaw-M_PI*2/3);
  //     armor3.y = target_position.y() - target.radius_1 * sin(target_yaw-M_PI*2/3);
  //     armor3.z = target_position.z() + target.d_zc;//当前装甲是较低的，dz>0，较高的dz<0
  //     armor3.yaw = target_yaw-M_PI*2/3;
  //     //printf("armor3_yaw %f \n",armor3.yaw);
  
  //     //将三块装甲板的数据存入vector
  //     current_armors.push_back(armor1);
  //     current_armors.push_back(armor2);
  //     current_armors.push_back(armor3);
  
  //   }

  //   best_idx =selectBestArmor(current_armors);

  //   if(best_idx==-1) break;

  //   Eigen::Vector3d best_armor(current_armors[best_idx].x,
  //                               current_armors[best_idx].y,
  //                               current_armors[best_idx].z);
  //   double flying_time2=trajectory_compensator_->getFlyingTime(best_armor);
  //   double new_dt=(current_time - rclcpp::Time(target.header.stamp)).seconds() + flying_time2 + prediction_delay_;

  //   std::cout<<"dt: "<<dt<<std::endl;

  //   std::cout<<"new_dt: "<<new_dt<<std::endl;

  //   if(std::fabs(new_dt-dt)<0.005) break;
  //   dt = (dt+new_dt)/2;


  // }

  
  //修正完毕，开始预测
  target_position.x() += dt * target.velocity.x*1.5;
  target_position.y() += dt * target.velocity.y*1.5;
  target_position.z() += dt * target.velocity.z*1.5;
  //std::cout<<"v_yaw"<<target.v_yaw<<std::endl;
  target_yaw += dt * target.v_yaw*1.62;//角速度比例系数

  //清空armors_data
  armors_data.clear();


  if(target.armors_num ==4){//r1是当前装甲板的轴向半径，
    // printf("armors_num %d",target_data.armors_num);
    // printf("\n");

    //计算四块装甲板的位置
    //target发布的
    ArmorData armor1;
    armor1.x = target_position.x() - target.radius_1 * cos(target_yaw);
    armor1.y = target_position.y() - target.radius_1 * sin(target_yaw);
    armor1.z = target_position.z() + target.d_zc;
    armor1.yaw = target_yaw;

    //target顺时针旋转90度的
    ArmorData armor2;
    armor2.x = target_position.x() - target.radius_2 * cos(target_yaw+M_PI/2);
    armor2.y = target_position.y() - target.radius_2 * sin(target_yaw+M_PI/2);
    armor2.z = target_position.z() + target.d_zc + target.d_za;//当前装甲是较低，dz>0，较高,dz<0
    armor2.yaw = target_yaw+M_PI/2;

    //target逆时针旋转90度的
    ArmorData armor3;
    armor3.x = target_position.x() - target.radius_2 * cos(target_yaw-M_PI/2);
    armor3.y = target_position.y() - target.radius_2 * sin(target_yaw-M_PI/2);
    armor3.z = target_position.z() + target.d_zc + target.d_za;//当前装甲是较低的，dz>0，较高的dz<0
    armor3.yaw = target_yaw-M_PI/2;


    //target顺时针旋转180度的
    ArmorData armor4;
    armor4.x = target_position.x() - target.radius_1 * cos(target_yaw+M_PI);
    armor4.y = target_position.y() - target.radius_1 * sin(target_yaw+M_PI);
    armor4.z = target_position.z() + target.d_zc;
    armor4.yaw = target_yaw+M_PI;

    //将四块装甲板的数据存入vector
    armors_data.push_back(armor1);
    armors_data.push_back(armor2);
    armors_data.push_back(armor3);
    armors_data.push_back(armor4);


  }

  else if (target.armors_num == 3)
  {
    // printf("armors_num %d",target_data.armors_num);
    // printf("\n");

    //printf("预测yaw预量: %f \n",float(yaw-target_data.yaw));
    //计算三块装甲板的位置
    //target发布的
    ArmorData armor1;
    armor1.x = target_position.x() - target.radius_1 * cos(target_yaw);
    armor1.y = target_position.y() - target.radius_1 * sin(target_yaw);
    armor1.z = target_position.z() + target.d_zc;
    armor1.yaw = target_yaw;
    //printf("armor1_yaw %f \n",armor1.yaw);

    //target顺时针旋转120度的
    ArmorData armor2;
    armor2.x = target_position.x() - target.radius_1 * cos(target_yaw+M_PI*2/3);
    armor2.y = target_position.y() - target.radius_1 * sin(target_yaw+M_PI*2/3);
    armor2.z = target_position.z() + target.d_zc;//当前装甲是较低的，dz>0，较高的dz<0
    armor2.yaw = target_yaw+M_PI*2/3;
    //printf("armor2_yaw %f \n",armor2.yaw);

    //target逆时针旋转120度的
    ArmorData armor3;
    armor3.x = target_position.x() - target.radius_1 * cos(target_yaw-M_PI*2/3);
    armor3.y = target_position.y() - target.radius_1 * sin(target_yaw-M_PI*2/3);
    armor3.z = target_position.z() + target.d_zc;//当前装甲是较低的，dz>0，较高的dz<0
    armor3.yaw = target_yaw-M_PI*2/3;
    //printf("armor3_yaw %f \n",armor3.yaw);

    //将三块装甲板的数据存入vector
    armors_data.push_back(armor1);
    armors_data.push_back(armor2);
    armors_data.push_back(armor3);

  }
  
  // Choose the best armor to shoot
  best_idx =selectBestArmor(armors_data);
  auto chosen_armor_position = armors_data[best_idx];//chosen_armor_position是armordata类
  Eigen::Vector3d chosen_armor_position_vector(chosen_armor_position.x,chosen_armor_position.y,chosen_armor_position.z);//描述装甲板的xyz向量

  if (std::sqrt(chosen_armor_position.x*chosen_armor_position.x
                +chosen_armor_position.y*chosen_armor_position.y
                +chosen_armor_position.z*chosen_armor_position.z) < 0.1) {
    throw std::runtime_error("No valid armor to shoot");
  }

  double yaw, pitch;//云台瞄准角度
  rm_interfaces::msg::GimbalCmd gimbal_cmd;

  //数字兵种
  if (target.armors_num == 4){

    // Calculate yaw, pitch, distance
    calcYawAndPitch(chosen_armor_position_vector, rpy_, yaw, pitch);//瞄准选好的装甲板
    double distance = std::sqrt(chosen_armor_position.x*chosen_armor_position.x+chosen_armor_position.y*chosen_armor_position.y);
    //std::cout<<"distance:"<<distance<<std::endl;
    // Initialize gimbal_cmd
    
    gimbal_cmd.header = target.header;
    gimbal_cmd.distance = distance;
    gimbal_cmd.fire_advice=false;//重置开火指令

    //std::cout<<"armor yaw "<<chosen_armor_position.yaw<<std::endl;
    // std::cout<<"计算pitch"<<pitch<<std::endl;
    // std::cout<<"电控pitch"<<rpy_[1]<<std::endl;
    // std::cout<<"计算yaw"<<yaw<<std::endl;
    // std::cout<<"电控yaw"<<rpy_[2]<<std::endl;


    //std::cout<<"pitch误差"<<std::fabs(pitch-rpy_[1])<<std::endl;
    std::cout<<"yaw误差"<<std::fabs(yaw-rpy_[2])<<std::endl;

    //std::cout<<"gimbal yaw "<<rpy_[2]<<std::endl;

    if(std::fabs(target.v_yaw)>8){
      gimbal_cmd.fire_advice=true;//对面转的太快，直接开火
    }

    else{
      //使用相对yaw角度，防止击打过于侧面的装甲板
      if(std::fabs(chosen_armor_position.yaw-rpy_[2])<0.2 ){

        if(isOnTarget(rpy_[2], rpy_[1], yaw, pitch, distance)){
          gimbal_cmd.fire_advice=true;
       }

      }
    }


    // Compensate angle by angle_offset_map
    auto angle_offset = manual_compensator_->angleHardCorrect(target_position.head(2).norm(), target_position.z());
    double pitch_offset = angle_offset[0] * M_PI / 180;
    double yaw_offset = angle_offset[1] * M_PI / 180;
    double cmd_pitch = pitch + pitch_offset;
    double cmd_yaw = angles::normalize_angle(yaw + yaw_offset);

    // gimbal_cmd.yaw = cmd_yaw * 180 / M_PI;
    // gimbal_cmd.pitch = cmd_pitch * 180 / M_PI; 
    gimbal_cmd.yaw = cmd_yaw;
    gimbal_cmd.pitch = cmd_pitch;
    gimbal_cmd.yaw_diff = (cmd_yaw - rpy_[2]) * 180 / M_PI;
    gimbal_cmd.pitch_diff = (cmd_pitch - rpy_[1]) * 180 / M_PI;


  }

  //前哨站
  else if (target.armors_num == 3){

    // Calculate yaw, pitch, distance
    calcYawAndPitch(chosen_armor_position_vector, rpy_, yaw, pitch);//瞄准中心
    double distance = std::sqrt(chosen_armor_position.x*chosen_armor_position.x+chosen_armor_position.y*chosen_armor_position.y);
    //std::cout<<"distance:"<<distance<<std::endl;
    // Initialize gimbal_cmd

    gimbal_cmd.header = target.header;
    gimbal_cmd.distance = distance;
    gimbal_cmd.fire_advice=false;//重置开火指令


    if(std::fabs(chosen_armor_position.yaw-rpy_[2])<0.6 ){
      //std::cout<<"计算pitch"<<pitch<<std::endl;
      //std::cout<<"电控pitch"<<rpy_[1]<<std::endl;

      gimbal_cmd.fire_advice=true;


    }

    // Compensate angle by angle_offset_map
    auto angle_offset = manual_compensator_->angleHardCorrect(target_position.head(2).norm(), target_position.z());
    double pitch_offset = angle_offset[0] * M_PI / 180;
    double yaw_offset = angle_offset[1] * M_PI / 180;
    double cmd_pitch = pitch + pitch_offset;
    double cmd_yaw = angles::normalize_angle(yaw + yaw_offset);


    // gimbal_cmd.yaw = cmd_yaw * 180 / M_PI;
    // gimbal_cmd.pitch = cmd_pitch * 180 / M_PI; 
    gimbal_cmd.yaw = cmd_yaw;
    gimbal_cmd.pitch = cmd_pitch;
    gimbal_cmd.yaw_diff = (cmd_yaw - rpy_[2]) * 180 / M_PI;
    gimbal_cmd.pitch_diff = (cmd_pitch - rpy_[1]) * 180 / M_PI;


  }

  
  // if (gimbal_cmd.fire_advice) {
  //   FYT_DEBUG("armor_solver", "You Need Fire!");
  // }
  return gimbal_cmd;
}





bool Solver::isOnTarget(const double cur_yaw,
                        const double cur_pitch,
                        const double target_yaw,
                        const double target_pitch,
                        const double distance) const noexcept {
  // Judge whether to shoot
  double shooting_range_yaw = std::abs(atan2(shooting_range_w_ / 2, distance));
  double shooting_range_pitch = std::abs(atan2(shooting_range_h_ / 2, distance));
  // Limit the shooting area to 1 degree to avoid not shooting when distance is
  // too largeselectBestArmor.0 * M_PI / 180);
  shooting_range_pitch = std::max(shooting_range_pitch, 1.0 * M_PI / 180);
  // shooting_range_yaw = std::max(shooting_range_yaw, 1.0 * M_PI / 180);
  if (std::abs(cur_yaw - target_yaw) < shooting_range_yaw &&
      std::abs(cur_pitch - target_pitch) < shooting_range_pitch) {
    return true;
  }

  return false;
}

// std::vector<Eigen::Vector3d> Solver::getArmorPositions(const Eigen::Vector3d &target_center,
//                                                        const double target_yaw,
//                                                        const double r1,
//                                                        const double r2,
//                                                        const double d_zc,
//                                                        const double d_za,
//                                                        const size_t armors_num) const noexcept {
//   auto armor_positions = std::vector<Eigen::Vector3d>(armors_num, Eigen::Vector3d::Zero());
//   // Calculate the position of each armor
//   bool is_current_pair = true;
//   double r = 0., target_dz = 0.;
//   for (size_t i = 0; i < armors_num; i++) {
//     double temp_yaw = target_yaw + i * (2 * M_PI / armors_num);
//     if (armors_num == 4) {
//       r = is_current_pair ? r1 : r2;
//       target_dz = d_zc + (is_current_pair ? 0 : d_za);
//       is_current_pair = !is_current_pair;
//     } else {
//       r = r1;
//       target_dz = d_zc;
//     }
//     armor_positions[i] =
//       target_center + Eigen::Vector3d(-r * cos(temp_yaw), -r * sin(temp_yaw), target_dz);
//   }
//   return armor_positions;
// }


int Solver::selectBestArmor(const std::vector<ArmorData>& armors_data) const noexcept {
  if (armors_data.empty()) {
    return -1;  // 空输入时返回无效索引
  }

  int selected_id = 0;
  float min_distance_sq = std::numeric_limits<float>::max();

  // 遍历所有装甲板，计算距离平方（避免开根号）
  for (size_t i = 0; i < armors_data.size(); ++i) {
    const auto& armor = armors_data[i];
    
    // 计算欧几里得距离平方（x² + y² + z²）
    const float dx = armor.x;
    const float dy = armor.y;
    const float dz = armor.z;
    const float distance_sq = dx * dx + dy * dy + dz * dz;

    // 更新最小距离和索引
    if (distance_sq < min_distance_sq) {
      min_distance_sq = distance_sq;
      selected_id = i;
    }
  }

  return selected_id;
}


void Solver::calcYawAndPitch(const Eigen::Vector3d &p,
                            const std::array<double, 3> rpy,
                            double &yaw,
                            double &pitch) const noexcept {
  // Calculate yaw and pitch
  yaw = atan2(p.y(), p.x());
  pitch = atan2(p.z(), p.head(2).norm());

  if (double temp_pitch = pitch; trajectory_compensator_->compensate(p, temp_pitch)) {
  pitch = temp_pitch;
  }
}

std::vector<std::pair<double, double>> Solver::getTrajectory() const noexcept {
  auto trajectory = trajectory_compensator_->getTrajectory(15, rpy_[1]);
  // Rotate
  for (auto &p : trajectory) {
    double x = p.first;
    double y = p.second;
    p.first = x * cos(rpy_[1]) + y * sin(rpy_[1]);
    p.second = -x * sin(rpy_[1]) + y * cos(rpy_[1]);
  }
  return trajectory;
}

}  // namespace fyt::auto_aim
