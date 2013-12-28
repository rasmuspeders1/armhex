/*
 * gait.cpp
 *
 *  Created on: Feb 1, 2013
 *      Author: rasmus
 */
#include "gait.h"

//Initialize pose to sane values
Pose::Pose():
body_x_(0),
body_y_(0),
body_z_(0),

body_roll_(0),
body_pitch_(0),
body_yaw_(0),
lf_limb_pos_(85.0, 75.0, 0.0),
lm_limb_pos_(0.0, 90.0, 0.0),
lr_limb_pos_(-85.0, 75.0, 0.0),
rf_limb_pos_(85.0, -75.0, 0.0),
rm_limb_pos_(0.0, -90.0, 0.0),
rr_limb_pos_(-85.0, -75.0, 0.0)
{

}

Pose::~Pose()
{

}



Gait::Gait():
step_lift_(20.0), //Default step lift is 30 mm
current_pose_(),
start_pose_(),
dest_pose_(),
//init pattern_ to basic ripple gait
//         LF    LM    LR    RR    RM    RF
pattern_{ {true ,false,false,false,false,false}, //First step
          {false,false,false,false,true ,false}, //Second step
          {false,false,true ,false,false,false}, //Third step
          {false,false,false,false,false,true }, //Fourth step
          {false,true ,false,false,false,false}, //Fifth step
          {false,false,false,true ,false,false}  //Sixth step
},
current_gait_cycle_step_(0),
gait_cycle_direction_(1)
{
  direction_vector_[0][0] = 0.0;
  direction_vector_[1][0] = 0.0;
  direction_vector_[2][0] = 0.0;
  direction_vector_[3][0] = 0.0;
}

Gait::~Gait()
{

}

void Gait::direction(kinematics::MatrixValue_t x, kinematics::MatrixValue_t y, kinematics::MatrixValue_t yaw)
{
  //Body destination yaw angle is the yaw angle at the beginning of gait cycle plus the input yaw
  dest_pose_.body_yaw_ = start_pose_.body_yaw_ + yaw;

  //Body destination position is the position at the start of the gait cycle plus the input.
  dest_pose_.body_x_ = start_pose_.body_x_ + x;
  dest_pose_.body_y_ = start_pose_.body_y_ + y;

  direction_vector_[0][0] = x;
  direction_vector_[1][0] = y;

  if(x < 0)
  {
    gait_cycle_direction_ = -1;
  }
  else
  {
    gait_cycle_direction_ = 1;
  }

  if(y < 0)
  {
    gait_cycle_start_ = 3;
  }
  else
  {
    gait_cycle_start_ = 0;
  }

//  if(direction_vector_.norm() > 0)
//  {
//    direction_vector_ = (direction_vector_ / direction_vector_.norm()) * 50.0;
//  }

  if(direction_vector_.norm() < 1.0)
  {
    return;
  }

  //Limb destination end points are the end point positions at the start if the gait cycle translated and rotated according to the input
  kinematics::BasicTransformationMatrix endpoint_transform(direction_vector_[0][0], direction_vector_[1][0], 0.0, 0.0, 0.0, yaw);
  kinematics::TranslationMatrix body_pos(start_pose_.body_x_, start_pose_.body_y_, start_pose_.body_z_);
  body_pos[3][0] = 0;

  Pose default_pose;

  dest_pose_.lf_limb_pos_ = endpoint_transform * (default_pose.lf_limb_pos_ + body_pos);
  dest_pose_.lm_limb_pos_ = endpoint_transform * (default_pose.lm_limb_pos_ + body_pos);
  dest_pose_.lr_limb_pos_ = endpoint_transform * (default_pose.lr_limb_pos_ + body_pos);
  dest_pose_.rf_limb_pos_ = endpoint_transform * (default_pose.rf_limb_pos_ + body_pos);
  dest_pose_.rm_limb_pos_ = endpoint_transform * (default_pose.rm_limb_pos_ + body_pos);
  dest_pose_.rr_limb_pos_ = endpoint_transform * (default_pose.rr_limb_pos_ + body_pos);


}

Pose Gait::target()
{

  bool increment_cycle = false;

  static bool still = true;

  static unsigned int old_gait_cycle_direction = gait_cycle_direction_;

  if(still && old_gait_cycle_direction != gait_cycle_direction_)
  {
    std::cout << "Gait cycle reverse!" << std::endl;
//    if(gait_cycle_direction_ > 0)
//    {
//      if(current_gait_cycle_step_ == (pattern_.size() - 1))
//      {
//        current_gait_cycle_step_ = 0;
//      }
//      else
//      {
//        current_gait_cycle_step_ += 1;
//      }
//    }
//    else
//    {
//      if(current_gait_cycle_step_ == 0)
//      {
//        current_gait_cycle_step_ = pattern_.size()-1;
//      }
//      else
//      {
//        current_gait_cycle_step_ -= 1;
//      }
//    }
    old_gait_cycle_direction = gait_cycle_direction_;
  }

  kinematics::MatrixValue_t end_point_min_dist = 50.0;

  if(gait_cycle_direction_ == 1)
  {
    kinematics::TranslationMatrix lm_lf_vec = current_pose_.lf_limb_pos_ - dest_pose_.lm_limb_pos_;
    while(lm_lf_vec.norm() < end_point_min_dist)
    {
      std::cout << "limiting dest" << std::endl;
      dest_pose_.lm_limb_pos_ = dest_pose_.lm_limb_pos_ - (lm_lf_vec / lm_lf_vec.norm());
      lm_lf_vec = current_pose_.lf_limb_pos_ - dest_pose_.lm_limb_pos_;
    }

    kinematics::TranslationMatrix lr_lm_vec = current_pose_.lm_limb_pos_ - dest_pose_.lr_limb_pos_;
    while(lr_lm_vec.norm() < end_point_min_dist)
    {
      dest_pose_.lr_limb_pos_ = dest_pose_.lr_limb_pos_ - (lr_lm_vec / lr_lm_vec.norm());
      lr_lm_vec = current_pose_.lm_limb_pos_ - dest_pose_.lr_limb_pos_;
    }

    kinematics::TranslationMatrix rm_rf_vec = current_pose_.rf_limb_pos_ - dest_pose_.rm_limb_pos_;
    while(rm_rf_vec.norm() < end_point_min_dist)
    {
      dest_pose_.rm_limb_pos_ = dest_pose_.rm_limb_pos_ - (rm_rf_vec / rm_rf_vec.norm());
      rm_rf_vec = current_pose_.rf_limb_pos_ - dest_pose_.rm_limb_pos_;
    }

    kinematics::TranslationMatrix rr_rm_vec = current_pose_.rm_limb_pos_ - dest_pose_.rr_limb_pos_;
    while(rr_rm_vec.norm() < end_point_min_dist)
    {
      dest_pose_.rr_limb_pos_ = dest_pose_.rr_limb_pos_ - (rr_rm_vec / rr_rm_vec.norm());
      rr_rm_vec = current_pose_.rm_limb_pos_ - dest_pose_.rr_limb_pos_;
    }

  }
  else if(gait_cycle_direction_ == -1)
  {
    kinematics::TranslationMatrix lm_lr_vec = current_pose_.lr_limb_pos_ - dest_pose_.lm_limb_pos_;
    while(lm_lr_vec.norm() < end_point_min_dist)
    {
      std::cout << "limiting dest" << std::endl;
      dest_pose_.lm_limb_pos_ = dest_pose_.lm_limb_pos_ - (lm_lr_vec / lm_lr_vec.norm());
      lm_lr_vec = current_pose_.lr_limb_pos_ - dest_pose_.lm_limb_pos_;
    }

    kinematics::TranslationMatrix lf_lm_vec = current_pose_.lm_limb_pos_ - dest_pose_.lf_limb_pos_;
    while(lf_lm_vec.norm() < end_point_min_dist)
    {
      dest_pose_.lf_limb_pos_ = dest_pose_.lf_limb_pos_ - (lf_lm_vec / lf_lm_vec.norm());
      lf_lm_vec = current_pose_.lm_limb_pos_ - dest_pose_.lf_limb_pos_;
    }

    kinematics::TranslationMatrix rm_rr_vec = current_pose_.rr_limb_pos_ - dest_pose_.rm_limb_pos_;
    while(rm_rr_vec.norm() < end_point_min_dist)
    {
      dest_pose_.rm_limb_pos_ = dest_pose_.rm_limb_pos_ - (rm_rr_vec / rm_rr_vec.norm());
      rm_rr_vec = current_pose_.rr_limb_pos_ - dest_pose_.rm_limb_pos_;
    }

    kinematics::TranslationMatrix rf_rm_vec = current_pose_.rm_limb_pos_ - dest_pose_.rf_limb_pos_;
    while(rf_rm_vec.norm() < end_point_min_dist)
    {
      dest_pose_.rf_limb_pos_ = dest_pose_.rf_limb_pos_ - (rf_rm_vec / rf_rm_vec.norm());
      rf_rm_vec = current_pose_.rm_limb_pos_ - dest_pose_.rf_limb_pos_;
    }
  }


  if(pattern_[current_gait_cycle_step_][0])
  {
    if((dest_pose_.lf_limb_pos_ - current_pose_.lf_limb_pos_).norm() > 0.01)
    {
      //if limb is lifted and has not reached its destination calculate next target end point
      current_pose_.lf_limb_pos_ = get_next_pos(start_pose_.lf_limb_pos_, dest_pose_.lf_limb_pos_, current_pose_.lf_limb_pos_);
      still = false;
    }
    else if(current_pose_.lf_limb_pos_ != start_pose_.lf_limb_pos_)
    {
      std::cout << "LF reached its destination" << std::endl;
      increment_cycle = true;
      still = true;
      start_pose_.lf_limb_pos_ = current_pose_.lf_limb_pos_;
      start_pose_.lf_limb_pos_.Print();
      current_pose_.lf_limb_pos_.Print();
    }
  }

  if(pattern_[current_gait_cycle_step_][1])
  {
    if((dest_pose_.lm_limb_pos_ - current_pose_.lm_limb_pos_).norm() > 0.01)
    {
      current_pose_.lm_limb_pos_ = get_next_pos(start_pose_.lm_limb_pos_, dest_pose_.lm_limb_pos_, current_pose_.lm_limb_pos_);
      still = false;
    }
    else if(current_pose_.lm_limb_pos_ != start_pose_.lm_limb_pos_)
    {
      std::cout << "LM reached its destination" << std::endl;
      increment_cycle = true;
      still = true;
      start_pose_.lm_limb_pos_ = current_pose_.lm_limb_pos_;
    }
  }

  if(pattern_[current_gait_cycle_step_][2])
  {
    if((dest_pose_.lr_limb_pos_ - current_pose_.lr_limb_pos_).norm() > 0.01)
    {
      current_pose_.lr_limb_pos_ = get_next_pos(start_pose_.lr_limb_pos_, dest_pose_.lr_limb_pos_, current_pose_.lr_limb_pos_);
      still = false;
    }
    else if(current_pose_.lr_limb_pos_ != start_pose_.lr_limb_pos_)
    {
      std::cout << "LR reached its destination" << std::endl;
      increment_cycle = true;
      still = true;
      start_pose_.lr_limb_pos_ = current_pose_.lr_limb_pos_;
    }
  }

  if(pattern_[current_gait_cycle_step_][3])
  {
    if((dest_pose_.rr_limb_pos_ - current_pose_.rr_limb_pos_).norm() > 0.01)
    {
      current_pose_.rr_limb_pos_ = get_next_pos(start_pose_.rr_limb_pos_, dest_pose_.rr_limb_pos_, current_pose_.rr_limb_pos_);
      still = false;
    }
    else if(current_pose_.rr_limb_pos_ != start_pose_.rr_limb_pos_)
    {
      std::cout << "RR reached its destination" << std::endl;
      increment_cycle = true;
      still = true;
      start_pose_.rr_limb_pos_ = current_pose_.rr_limb_pos_;
    }
  }

  if(pattern_[current_gait_cycle_step_][4])
  {
    if((dest_pose_.rm_limb_pos_ - current_pose_.rm_limb_pos_).norm() > 0.01)
    {
      current_pose_.rm_limb_pos_ = get_next_pos(start_pose_.rm_limb_pos_, dest_pose_.rm_limb_pos_, current_pose_.rm_limb_pos_);
      still = false;
    }
    else if(current_pose_.rm_limb_pos_ != start_pose_.rm_limb_pos_)
    {
      std::cout << "RM reached its destination" << std::endl;
      increment_cycle = true;
      still = true;
      start_pose_.rm_limb_pos_ = current_pose_.rm_limb_pos_;
    }
  }

  if(pattern_[current_gait_cycle_step_][5])
  {
    if((dest_pose_.rf_limb_pos_ - current_pose_.rf_limb_pos_).norm() > 0.01)
    {
      current_pose_.rf_limb_pos_ = get_next_pos(start_pose_.rf_limb_pos_, dest_pose_.rf_limb_pos_, current_pose_.rf_limb_pos_);
      still = false;
    }
    else if(current_pose_.rf_limb_pos_ != start_pose_.rf_limb_pos_)
    {
      std::cout << "RF reached its destination" << std::endl;
      increment_cycle = true;
      still = true;
      start_pose_.rf_limb_pos_ = current_pose_.rf_limb_pos_;
    }
  }

  center_body();

  if(increment_cycle)
  {
    start_pose_ = current_pose_;
    std::cout << "Gait cycle to step: " << current_gait_cycle_step_ << "\nGait cycle direction: " << (gait_cycle_direction_ > 0 ? "Forward" : "Reverse") << std::endl;
    if(gait_cycle_direction_ > 0)
    {
      if(current_gait_cycle_step_ == (pattern_.size() - 1))
      {
        current_gait_cycle_step_ = 0;
      }
      else
      {
        current_gait_cycle_step_ += 1;
      }
    }
    else
    {
      if(current_gait_cycle_step_ == 0)
      {
        current_gait_cycle_step_ = pattern_.size()-1;
      }
      else
      {
        current_gait_cycle_step_ -= 1;
      }
    }
  }


  return current_pose_;
}

kinematics::TranslationMatrix Gait::get_next_pos(const kinematics::TranslationMatrix &start, const kinematics::TranslationMatrix &dest, const kinematics::TranslationMatrix &current)
{
  //Direction vector for limb end point
  kinematics::TranslationMatrix dir_vec = dest - current;

  //normalized direction vector. Not normalized if norm smaller than 1
  kinematics::TranslationMatrix norm_dir_vec = (dir_vec / dir_vec.norm());

  //calculate limb lift height from new target x,y position
  kinematics::MatrixValue_t step_length = (dest - start).norm();


  kinematics::TranslationMatrix target_pos;

  if(dir_vec.norm() > 1.0)
  {
    target_pos[0][0] = current[0][0] + norm_dir_vec[0][0];
    target_pos[1][0] = current[1][0] + norm_dir_vec[1][0];

    if(step_length > 0)
    {
      kinematics::MatrixValue_t limb_z = fabs(sin(((step_length - (dest - target_pos).norm() ) / step_length) * M_PI)) * step_lift_;
      target_pos[2][0] = limb_z;
    }
    else
    {
      target_pos[2][0] = current[2][0] > 1 ? current[2][0] - 1 : 0.0;
    }

    //Calculate new normalized path vector and add to current position of limb
    kinematics::TranslationMatrix movement_delta = target_pos - current;
    movement_delta = dir_vec.norm() > 3.0 ? (movement_delta / movement_delta.norm()) * 3.0 : movement_delta;
    kinematics::TranslationMatrix next = current + movement_delta;
    //Make sure "scale" in translation matrix is 1
    next[3][0] = 1;
    return next;
  }
  else
  {
    return dest;
  }

}

void Gait::center_body()
{
	//For now this is implemented as average of all leg end points
  current_pose_.body_x_ = (current_pose_.lf_limb_pos_[0][0]
                         + current_pose_.lm_limb_pos_[0][0]
                         + current_pose_.lr_limb_pos_[0][0]
                         + current_pose_.rr_limb_pos_[0][0]
                         + current_pose_.rm_limb_pos_[0][0]
                         + current_pose_.rf_limb_pos_[0][0])
                         / 6.0;

  current_pose_.body_y_ = (current_pose_.lf_limb_pos_[1][0]
                         + current_pose_.lm_limb_pos_[1][0]
                         + current_pose_.lr_limb_pos_[1][0]
                         + current_pose_.rr_limb_pos_[1][0]
                         + current_pose_.rm_limb_pos_[1][0]
                         + current_pose_.rf_limb_pos_[1][0])
                         / 6.0;
  //For now this is implemented as average of all leg end points
  current_pose_.body_z_ = (current_pose_.lf_limb_pos_[2][0]
                         + current_pose_.lm_limb_pos_[2][0]
                         + current_pose_.lr_limb_pos_[2][0]
                         + current_pose_.rr_limb_pos_[2][0]
                         + current_pose_.rm_limb_pos_[2][0]
                         + current_pose_.rf_limb_pos_[2][0])
                         / 6.0;

//#Calculate body rotation relative to feet positions
//#The yaw of the body must be in line with the polygon spanned by the feet positions.
//left_dir_vec = (self.feet_pos['lf'] - self.feet_pos['lb'])
//right_dir_vec = (self.feet_pos['rf'] - self.feet_pos['rb'])
//mean_dir_vec = numpy.mean(numpy.hstack([left_dir_vec, right_dir_vec]), 1)
//
//body_yaw = degrees(atan2(mean_dir_vec[1], mean_dir_vec[0]))
  //calculate body yaw angle from limb end point positions
  kinematics::TranslationMatrix left_dir_vec = current_pose_.lf_limb_pos_ - current_pose_.lr_limb_pos_;
  kinematics::TranslationMatrix right_dir_vec = current_pose_.rf_limb_pos_ - current_pose_.rr_limb_pos_;
  kinematics::TranslationMatrix mean_dir_vec = (left_dir_vec + right_dir_vec) / 2.0;
//  mean_dir_vec.Print();
  current_pose_.body_yaw_= atan2(mean_dir_vec[1][0], mean_dir_vec[0][0]);
//  std::cout << "YAW: " << current_pose_.body_yaw_ * (180.0/M_PI) << std::endl;

}





