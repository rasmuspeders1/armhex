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

  Link::Link()
  {
  }

  Link::Link(Value_t thetaIn, Value_t alphaIn, Value_t aIn, Value_t dIn,
      const std::string &linkNameIn, int servoAddrIn) :
      theta_(thetaIn), alpha_(alphaIn), a_(aIn), d_(dIn), link_name_(
          linkNameIn), servo_addr_(servoAddrIn)
  {
    std::cout << "Link <" << link_name_ << "> created" << std::endl;

    cos_alpha_ = cos(alpha_ / (180.0 / M_1_PI));
    sin_alpha_ = sin(alpha_ / (180.0 / M_1_PI));
    cos_theta_ = cos(theta_ / (180.0 / M_1_PI));
    sin_theta_ = sin(theta_ / (180.0 / M_1_PI));

  }

  const TransformationMatrix & Link::getTransformationMatrix(Value_t theta)
  {
    set_theta(theta);
    return getTransformationMatrix();
  }

  const TransformationMatrix & Link::getTransformationMatrix()
  {
    //Calculate Tranformation matrix for this link with the current theta angle

    //First Row
    transformation_matrix_[0][0] = cos_theta_;
    transformation_matrix_[0][1] = -sin_theta_ * cos_alpha_;
    transformation_matrix_[0][2] = sin_theta_ * sin_alpha_;
    transformation_matrix_[0][3] = a_ * cos_theta_;

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

    return transformation_matrix_;
  }

  Link::~Link()
  {

  }

  Limb::Limb()
  {

  }

//  {'lf':numpy.matrix([[  85.0], [  75.0], [   0.0]]),
//                          'lm':numpy.matrix([[   0.0], [  90.0], [   0.0]]),
//                          'lb':numpy.matrix([[ -85.0], [  75.0], [   0.0]]),
//                          'rf':numpy.matrix([[  85.0], [ -75.0], [   0.0]]),
//                          'rm':numpy.matrix([[   0.0], [ -90.0], [   0.0]]),
//                          'rb':numpy.matrix([[ -85.0], [ -75.0], [   0.0]])

  Limb::Limb(Body bodyIn, Link bodyLinkIn, Link coxaLinkIn, Link femurLinkIn,
      Link tibiaLinkIn)
  {
    body_ = bodyIn;
    body_link_ = bodyLinkIn;
    coxa_link_ = coxaLinkIn;
    femur_link_ = femurLinkIn;
    tibia_link_ = tibiaLinkIn;

    //TODO: calculate safe initial position from body and link information. Do this with normal forward kinematics. Need transformation matrix for body.

  }

  bool Limb::InverseKinematic(Value_t x, Value_t y, Value_t z)
  {
    //Translation Matrix to this point

    return true;
  }

  Limb::~Limb()
  {

  }

  Body::Body()
  {

  }

  Body::~Body()
  {

  }

}
