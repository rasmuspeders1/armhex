/*
 * kinematics.c++
 *
 *  Created on: Jul 14, 2012
 *      Author: rasmus
 */

#include "kinematics.h"
#include <iostream>

namespace kinematics
{
  /*
   * General matrix Implementations
   */
  TransformationMatrix::TransformationMatrix() :
      Matrix(4, 4)
  {

  }

  TransformationMatrix::~TransformationMatrix()
  {

  }

  TranslationMatrix::TranslationMatrix() :
      Matrix(4, 1)
  {

  }

  TranslationMatrix::~TranslationMatrix()
  {

  }

  RotationMatrix::RotationMatrix() :
      Matrix(3, 3)
  {

  }

  RotationMatrix::~RotationMatrix()
  {

  }

  /*
   * Link Class Implementation
   */
  Link::Link()
  {
  }

  Link::Link(MatrixValue thetaIn, MatrixValue alphaIn, MatrixValue aIn, MatrixValue dIn, std::string linkNameIn, int servoAddrIn) :
      theta_(thetaIn), alpha_(alphaIn), a_(aIn), d_(dIn), link_name_(
          linkNameIn), servo_addr_(servoAddrIn)
  {
    std::cout << "Link <" << link_name_ << "> created" << std::endl;

    cos_alpha_ = cos(alpha_);
    sin_alpha_ = sin(alpha_);
    cos_theta_ = cos(theta_);
    sin_theta_ = sin(theta_);

  }

  const TransformationMatrix & Link::update_transformation_matrix(MatrixValue theta)
  {
    theta_ = theta;
    cos_theta_ = cos(theta_);
    sin_theta_ = sin(theta_);

    //First Row
    transformation_matrix_[0][0] = cos_theta_;
    transformation_matrix_[1][0] = -sin_theta_ * cos_alpha_;
    transformation_matrix_[2][0] = sin_theta_ * sin_alpha_;
    transformation_matrix_[3][0] = a_ * cos_theta_;

    //Second Row
    transformation_matrix_[1][0] = sin_theta_;
    transformation_matrix_[1][1] = cos_theta_ * cos_alpha_;
    transformation_matrix_[1][2] = -cos_theta_ * sin_alpha_;
    transformation_matrix_[1][3] = a_ * sin_theta_;

    //Third Row
    transformation_matrix_[2][0] = 0;
    transformation_matrix_[2][1] = sin_alpha_ * cos_theta_;
    transformation_matrix_[2][2] = sin_theta_ * sin_alpha_;
    transformation_matrix_[2][3] = d_;

    //Fourth Row
    transformation_matrix_[3][0] = 0;
    transformation_matrix_[3][1] = 0;
    transformation_matrix_[3][2] = 0;
    transformation_matrix_[3][3] = 1;

    return get_transformation_matrix();
  }

  const TransformationMatrix & Link::get_transformation_matrix()
  {


    return transformation_matrix_;
  }

  Link::~Link()
  {

  }

//  {'lf':numpy.matrix([[  85.0], [  75.0], [   0.0]]),
//                          'lm':numpy.matrix([[   0.0], [  90.0], [   0.0]]),
//                          'lb':numpy.matrix([[ -85.0], [  75.0], [   0.0]]),
//                          'rf':numpy.matrix([[  85.0], [ -75.0], [   0.0]]),
//                          'rm':numpy.matrix([[   0.0], [ -90.0], [   0.0]]),
//                          'rb':numpy.matrix([[ -85.0], [ -75.0], [   0.0]])

  /*
   * Limb Implementation
   */
  Limb::Limb(const Body &body, Link &body_link, Link &coxa_link, Link &femur_link, Link &tibia_link)
  {
    body_ = body;
    body_link_ = body_link;
    coxa_link_ = coxa_link;
    femur_link_ = femur_link;
    tibia_link_ = tibia_link;

    //TODO: calculate safe initial position from body and link information. Do this with normal forward kinematics. Need transformation matrix for body.

  }

  bool Limb::InverseKinematic(MatrixValue x, MatrixValue y, MatrixValue z)
  {
    //Translation Matrix to this point

    return true;
  }

  Limb::~Limb()
  {

  }

  /*
   * Body Implementation
   */
  Body::Body()
  {

  }

  Body::~Body()
  {

  }

  const TransformationMatrix & Body::update_transformation_matrix(MatrixValue roll, MatrixValue pitch, MatrixValue yaw, MatrixValue x, MatrixValue y, MatrixValue z)
  {
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
    x_ = x;
    y_ = y;
    z_ = z;

    cos_roll_ = cos(roll_);
    cos_pitch_ = cos(pitch_);
    cos_yaw_ = cos(yaw_);

    sin_roll_ = sin(roll_);
    sin_pitch_ = sin(pitch_);
    sin_yaw_ = sin(yaw_);

    //First Column
    transformation_matrix_[0][0] = cos_yaw_ * cos_pitch_;
    transformation_matrix_[1][0] = sin_yaw_ * cos_pitch_;
    transformation_matrix_[2][0] = -sin_pitch_;
    transformation_matrix_[3][0] = 0;

    //Second Row
    transformation_matrix_[0][1] = -cos_roll_ * sin_yaw_ + cos_yaw_ * sin_roll_ * sin_pitch_;
    transformation_matrix_[1][1] = cos_roll_ * cos_yaw_ + sin_roll_ * sin_pitch_ * sin_yaw_;
    transformation_matrix_[2][1] = cos_pitch_ * sin_roll_;
    transformation_matrix_[3][1] = 0;

    //Third Row
    transformation_matrix_[0][2] = sin_roll_ * sin_yaw_ + cos_roll_ * cos_yaw_ * sin_pitch_;
    transformation_matrix_[1][2] = -cos_yaw_ * sin_roll_ + cos_roll_ * sin_pitch_ * sin_yaw_;
    transformation_matrix_[2][2] = cos_roll_ * cos_pitch_;
    transformation_matrix_[3][2] = 0;

    //Fourth Row
    transformation_matrix_[0][3] = x_;
    transformation_matrix_[1][3] = y_;
    transformation_matrix_[2][3] = z_;
    transformation_matrix_[3][3] = 1;

    return transformation_matrix_;
  }

  const TransformationMatrix & Body::get_transformation_matrix()
  {
    return transformation_matrix_;
  }

}
