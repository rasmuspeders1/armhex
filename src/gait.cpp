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

kinematics::TranslationMatrix Pose::GetBodyPos()
{
  return kinematics::TranslationMatrix(body_x_, body_y_, body_z_);
}

void Pose::SetBodyPos(const kinematics::TranslationMatrix body_pos)
{
  body_x_ = body_pos[0][0];
  body_y_ = body_pos[1][0];
  body_z_ = body_pos[2][0];
}

void Pose::Print()
{
  std::cout << "Pose:" << std::endl;
  std::cout << "Body X: " << body_x_ << std::endl;
  std::cout << "Body Y: " << body_y_ << std::endl;
  std::cout << "Body Z: " << body_z_ << std::endl;
  std::cout << "Body Roll: " << body_roll_ << std::endl;
  std::cout << "Body Pitch: " << body_pitch_ << std::endl;
  std::cout << "Body Yaw: " << body_yaw_ << std::endl;

  std::cout << "LF Limb Pos:" << std::endl;
  lf_limb_pos_.Print();
  std::cout << "LM Limb Pos:" << std::endl;
  lm_limb_pos_.Print();
  std::cout << "LR Limb Pos:" << std::endl;
  lr_limb_pos_.Print();
  std::cout << "RF Limb Pos:" << std::endl;
  rf_limb_pos_.Print();
  std::cout << "RM Limb Pos:" << std::endl;
  rm_limb_pos_.Print();
  std::cout << "RR Limb Pos:" << std::endl;
  rr_limb_pos_.Print();
}

Gait::Gait():
step_lift_(20.0), //Default step lift is 30 mm
current_pose_(),
start_pose_(),
dest_pose_(),
//init pattern_
//         LF    LM    LR    RR    RM    RF
pattern_{ {true ,false,false,false,false,false}, //First step
          {false,false,false,true ,false,false}, //Second step
          {false,true ,false,false,false,false}, //Third step
          {false,false,false,false,false,true }, //Fourth step
          {false,false,true ,false,false,false}, //Fifth step
          {false,false,false,false,true ,false}, //Sixth step
},

current_gait_cycle_step_(0),
gait_cycle_direction_(1),
gait_cycle_start_(0),
body_relative_x_(0),
body_relative_y_(0),
body_relative_z_(0),
body_relative_roll_(0),
body_relative_pitch_(0),
body_relative_yaw_(0)
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

  if(direction_vector_.norm() < 1.0 && fabs(yaw) < 0.1)
  {
    return;
  }

  //calculate
  kinematics::TranslationMatrix start_center;

  start_center[0][0] = (start_pose_.lf_limb_pos_[0][0]
                       + start_pose_.lm_limb_pos_[0][0]
                       + start_pose_.lr_limb_pos_[0][0]
                       + start_pose_.rr_limb_pos_[0][0]
                       + start_pose_.rm_limb_pos_[0][0]
                       + start_pose_.rf_limb_pos_[0][0])
                       / 6.0;


  start_center[1][0] = (start_pose_.lf_limb_pos_[1][0]
                       + start_pose_.lm_limb_pos_[1][0]
                       + start_pose_.lr_limb_pos_[1][0]
                       + start_pose_.rr_limb_pos_[1][0]
                       + start_pose_.rm_limb_pos_[1][0]
                       + start_pose_.rf_limb_pos_[1][0])
                       / 6.0;

  start_center[2][0] = 0;
  start_center[3][0] = 0;

  //Calculate body rotation relative to feet positions
  //The yaw of the body must be in line with the polygon spanned by the feet positions.

  kinematics::TranslationMatrix left_dir_vec = start_pose_.lf_limb_pos_ - start_pose_.lr_limb_pos_;
  kinematics::TranslationMatrix right_dir_vec = start_pose_.rf_limb_pos_ - start_pose_.rr_limb_pos_;
  kinematics::TranslationMatrix mean_dir_vec = (left_dir_vec + right_dir_vec) / 2.0;
  kinematics::MatrixValue_t start_gait_yaw_= atan2(mean_dir_vec[1][0], mean_dir_vec[0][0]);

  //Limb destination end points are the end point positions at the start if the gait cycle translated and rotated according to the input

  //First create transformation matrix for moving limb polygon according to gait input from user from body frame
  kinematics::BasicTransformationMatrix gait_transform(direction_vector_[0][0], direction_vector_[1][0], 0.0, 0.0, 0.0, start_gait_yaw_ + yaw);
  //Then create transformation matrix to transform to limb polygon frame
  kinematics::BasicTransformationMatrix limb_polygon_transform(start_center[0][0], start_center[1][0], 0.0, 0.0, 0.0, start_gait_yaw_);
  //Then create transformation for gait polygon movement.
  //First transform to body frame, then do gait tranformation and transform back to global coordinates.
  kinematics::BasicTransformationMatrix endpoint_transform = limb_polygon_transform * gait_transform * limb_polygon_transform.InverseTransformation();

  Pose default_pose;

  dest_pose_.lf_limb_pos_ = endpoint_transform * (default_pose.lf_limb_pos_ + start_center);
  dest_pose_.lm_limb_pos_ = endpoint_transform * (default_pose.lm_limb_pos_ + start_center);
  dest_pose_.lr_limb_pos_ = endpoint_transform * (default_pose.lr_limb_pos_ + start_center);
  dest_pose_.rf_limb_pos_ = endpoint_transform * (default_pose.rf_limb_pos_ + start_center);
  dest_pose_.rm_limb_pos_ = endpoint_transform * (default_pose.rm_limb_pos_ + start_center);
  dest_pose_.rr_limb_pos_ = endpoint_transform * (default_pose.rr_limb_pos_ + start_center);


}

void Gait::body_relative_position(kinematics::MatrixValue_t x, kinematics::MatrixValue_t y, kinematics::MatrixValue_t z)
{
  body_relative_x_ = x;
  body_relative_y_ = y;
  body_relative_z_ = z;
}

void Gait::body_relative_rotation(kinematics::MatrixValue_t roll, kinematics::MatrixValue_t pitch, kinematics::MatrixValue_t yaw)
{
  body_relative_roll_ = roll;
  body_relative_pitch_ = pitch;
  body_relative_yaw_ = yaw;
}

Pose Gait::target()
{

  bool increment_cycle = true;

  if(pattern_[current_gait_cycle_step_][0])
  {
    if((dest_pose_.lf_limb_pos_ - current_pose_.lf_limb_pos_).norm() > 0.01)
    {
      //if limb is lifted and has not reached its destination calculate next target end point
      current_pose_.lf_limb_pos_ = get_next_pos(start_pose_.lf_limb_pos_, dest_pose_.lf_limb_pos_, current_pose_.lf_limb_pos_);
      increment_cycle = false;
    }
    else if(current_pose_.lf_limb_pos_ != start_pose_.lf_limb_pos_)
    {
      std::cout << "LF reached its destination" << std::endl;
    }
  }

  if(pattern_[current_gait_cycle_step_][1])
  {
    if((dest_pose_.lm_limb_pos_ - current_pose_.lm_limb_pos_).norm() > 0.01)
    {
      current_pose_.lm_limb_pos_ = get_next_pos(start_pose_.lm_limb_pos_, dest_pose_.lm_limb_pos_, current_pose_.lm_limb_pos_);
      increment_cycle = false;
    }
    else if(current_pose_.lm_limb_pos_ != start_pose_.lm_limb_pos_)
    {
      std::cout << "LM reached its destination" << std::endl;
    }
  }

  if(pattern_[current_gait_cycle_step_][2])
  {
    if((dest_pose_.lr_limb_pos_ - current_pose_.lr_limb_pos_).norm() > 0.01)
    {
      current_pose_.lr_limb_pos_ = get_next_pos(start_pose_.lr_limb_pos_, dest_pose_.lr_limb_pos_, current_pose_.lr_limb_pos_);
      increment_cycle = false;
    }
    else if(current_pose_.lr_limb_pos_ != start_pose_.lr_limb_pos_)
    {
      std::cout << "LR reached its destination" << std::endl;
    }
  }

  if(pattern_[current_gait_cycle_step_][3])
  {
    if((dest_pose_.rr_limb_pos_ - current_pose_.rr_limb_pos_).norm() > 0.01)
    {
      current_pose_.rr_limb_pos_ = get_next_pos(start_pose_.rr_limb_pos_, dest_pose_.rr_limb_pos_, current_pose_.rr_limb_pos_);
      increment_cycle = false;
    }
    else if(current_pose_.rr_limb_pos_ != start_pose_.rr_limb_pos_)
    {
      std::cout << "RR reached its destination" << std::endl;
    }
  }

  if(pattern_[current_gait_cycle_step_][4])
  {
    if((dest_pose_.rm_limb_pos_ - current_pose_.rm_limb_pos_).norm() > 0.01)
    {
      current_pose_.rm_limb_pos_ = get_next_pos(start_pose_.rm_limb_pos_, dest_pose_.rm_limb_pos_, current_pose_.rm_limb_pos_);
      increment_cycle = false;
    }
    else if(current_pose_.rm_limb_pos_ != start_pose_.rm_limb_pos_)
    {
      std::cout << "RM reached its destination" << std::endl;
    }
  }

  if(pattern_[current_gait_cycle_step_][5])
  {
    if((dest_pose_.rf_limb_pos_ - current_pose_.rf_limb_pos_).norm() > 0.01)
    {
      current_pose_.rf_limb_pos_ = get_next_pos(start_pose_.rf_limb_pos_, dest_pose_.rf_limb_pos_, current_pose_.rf_limb_pos_);
      increment_cycle = false;
    }
    else if(current_pose_.rf_limb_pos_ != start_pose_.rf_limb_pos_)
    {
      std::cout << "RF reached its destination" << std::endl;
    }
  }

  //make sure the body follows the limbs
  center_body();

  if(increment_cycle)
  {
    start_pose_ = current_pose_; //set new start pose as current pose

    //increment gait cycle step according to gait cycle direction
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

    std::cout << "Gait cycle to step: " << current_gait_cycle_step_ << "\nGait cycle direction: " << (gait_cycle_direction_ > 0 ? "Forward" : "Reverse") << std::endl;
  }

  current_pose_.Print();
  return current_pose_;
}

kinematics::TranslationMatrix Gait::get_next_pos(kinematics::TranslationMatrix &start, const kinematics::TranslationMatrix &dest, const kinematics::TranslationMatrix &current)
{
  //Direction vector for limb end point. From current position to destination position
  kinematics::TranslationMatrix dir_vec = dest - current;

  //normalized direction vector.
  kinematics::TranslationMatrix norm_dir_vec = (dir_vec / dir_vec.norm());

  //calculate limb lift height from new target x,y position
  kinematics::MatrixValue_t step_length = (dest - start).norm();

  if(dir_vec.norm() > step_length)
  {
    start = current;
    dir_vec = dest - current;
    step_length = (dest - start).norm();
  }

  kinematics::TranslationMatrix target_pos;

  //Only use normalized vector if direction vector norm is larger than 1.
  //otherwise go directly to destination
  if(dir_vec.norm() <= 1.0)
  {
    return dest;
  }

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

void Gait::center_body()
{
  //For now this is implemented as average of all leg end points plus relative translation
  kinematics::TranslationMatrix body_dest;
  body_dest[0][0] = (current_pose_.lf_limb_pos_[0][0]
                     + current_pose_.lm_limb_pos_[0][0]
                     + current_pose_.lr_limb_pos_[0][0]
                     + current_pose_.rr_limb_pos_[0][0]
                     + current_pose_.rm_limb_pos_[0][0]
                     + current_pose_.rf_limb_pos_[0][0])
                     / 6.0;

  body_dest[0][0] += body_relative_x_;


  body_dest[1][0] = (current_pose_.lf_limb_pos_[1][0]
                         + current_pose_.lm_limb_pos_[1][0]
                         + current_pose_.lr_limb_pos_[1][0]
                         + current_pose_.rr_limb_pos_[1][0]
                         + current_pose_.rm_limb_pos_[1][0]
                         + current_pose_.rf_limb_pos_[1][0])
                         / 6.0;

  body_dest[1][0] += body_relative_y_;

//  //For now this is implemented as average of all leg end points
//  body_dest[2][0] = (current_pose_.lf_limb_pos_[2][0]
//                         + current_pose_.lm_limb_pos_[2][0]
//                         + current_pose_.lr_limb_pos_[2][0]
//                         + current_pose_.rr_limb_pos_[2][0]
//                         + current_pose_.rm_limb_pos_[2][0]
//                         + current_pose_.rf_limb_pos_[2][0])
//                         / 6.0;

  body_dest[2][0] += body_relative_z_;

  //Scale of body dest must be zero
  body_dest[3][0] = 0;

  kinematics::TranslationMatrix current_body_pos = current_pose_.GetBodyPos();
  //scale of body pos must also be zero
  current_body_pos[3][0] = 0;

  kinematics::TranslationMatrix body_dir_vec = body_dest - current_body_pos;
  kinematics::TranslationMatrix norm_body_dir_vec = body_dir_vec / body_dir_vec.norm();
  std::cout << "Body DIR Vec" << std::endl;
  norm_body_dir_vec.Print();

  if(body_dir_vec.norm() < 1.0)
  {
    current_pose_.SetBodyPos(body_dest);
  }
  else
  {
    current_pose_.SetBodyPos(current_body_pos + norm_body_dir_vec);
  }

  current_pose_.body_roll_ = body_relative_roll_;
  current_pose_.body_pitch_ = body_relative_pitch_;

  //Calculate body rotation relative to feet positions
  //The yaw of the body must be in line with the polygon spanned by the feet positions.

  kinematics::TranslationMatrix left_dir_vec = current_pose_.lf_limb_pos_ - current_pose_.lr_limb_pos_;
  kinematics::TranslationMatrix right_dir_vec = current_pose_.rf_limb_pos_ - current_pose_.rr_limb_pos_;
  kinematics::TranslationMatrix mean_dir_vec = (left_dir_vec + right_dir_vec) / 2.0;
//  mean_dir_vec.Print();
  current_pose_.body_yaw_= atan2(mean_dir_vec[1][0], mean_dir_vec[0][0]);

  current_pose_.body_yaw_ += body_relative_yaw_;
//  std::cout << "YAW: " << current_pose_.body_yaw_ * (180.0/M_PI) << std::endl;

}





