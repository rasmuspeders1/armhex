/*
 * kinematics.c++
 *
 *  Created on: Jul 14, 2012
 *      Author: rasmus
 */

#include "kinematics.h"
#include <iostream>
#include <iomanip>

namespace kinematics
{

  void Matrix::Print() const
  {
    for (unsigned int row = 0; row < rows_; row++)
    {
      for (unsigned int col = 0; col < columns_; col++)
      {
        std::cout << std::setprecision(5) << std::setw(11) << std::setfill(' ')
            << data[row][col];
        if(col == columns_ - 1)
        {
          std::cout << std::endl;
        }
        else
        {
          std::cout << " ";
        }
      }
    }
  }

  /*
   * General matrix Implementations
   */
  TransformationMatrix::TransformationMatrix() :
      Matrix(4, 4)
  {

  }

  TransformationMatrix::TransformationMatrix(Matrix other) :
      Matrix(other)
  {

  }

  TransformationMatrix::~TransformationMatrix()
  {

  }

  TranslationMatrix TransformationMatrix::Translation() const
  {
    TranslationMatrix translation;
    translation[0][0] = (*this)[0][3];
    translation[1][0] = (*this)[1][3];
    translation[2][0] = (*this)[2][3];
    translation[3][0] = (*this)[3][3];
    return translation;
  }

  RotationMatrix TransformationMatrix::Rotation() const
  {
    RotationMatrix rotation;
    rotation[0][0] = (*this)[0][0];
    rotation[0][1] = (*this)[0][1];
    rotation[0][2] = (*this)[0][2];

    rotation[1][0] = (*this)[1][0];
    rotation[1][1] = (*this)[1][1];
    rotation[1][2] = (*this)[1][2];

    rotation[2][0] = (*this)[2][0];
    rotation[2][1] = (*this)[2][1];
    rotation[2][2] = (*this)[2][2];
    return rotation;
  }

  TransformationMatrix TransformationMatrix::InverseTransformation() const
  {
    TransformationMatrix inverse_transformation = TransformationMatrix();

    RotationMatrix rotation_transpose(Rotation().Transpose());
    inverse_transformation[0][0] = rotation_transpose[0][0];
    inverse_transformation[0][1] = rotation_transpose[0][1];
    inverse_transformation[0][2] = rotation_transpose[0][2];

    inverse_transformation[1][0] = rotation_transpose[1][0];
    inverse_transformation[1][1] = rotation_transpose[1][1];
    inverse_transformation[1][2] = rotation_transpose[1][2];

    inverse_transformation[2][0] = rotation_transpose[2][0];
    inverse_transformation[2][1] = rotation_transpose[2][1];
    inverse_transformation[2][2] = rotation_transpose[2][2];

    //TODO: Add operator -/+ for Matrix class
    TransformationMatrix translation = Translation();
    Matrix translation_vector = Matrix(3, 1);
    translation_vector[0][0] = translation[0][0];
    translation_vector[1][0] = translation[1][0];
    translation_vector[2][0] = translation[2][0];

    TranslationMatrix inverse_translation = rotation_transpose * translation_vector;

    inverse_transformation[0][3] = -inverse_translation[0][0];
    inverse_transformation[1][3] = -inverse_translation[1][0];
    inverse_transformation[2][3] = -inverse_translation[2][0];
    inverse_transformation[3][3] = 1;

    return inverse_transformation;

//    rMatrix = self.getRMatrix()
//
//    translationMatrix = self.getTranslationMatrix()
//
//    iTMatrix = rMatrix.transpose()
//
//    matrix2 = -(rMatrix.transpose() * translationMatrix)
//
//    iTMatrix = numpy.append(iTMatrix, matrix2, axis=1)
//    iTMatrix = numpy.append(iTMatrix, numpy.matrix([0, 0, 0, 1]), axis=0)
//
//    return iTMatrix
  }

  TranslationMatrix::TranslationMatrix() :
      Matrix(4, 1)
  {

  }

  TranslationMatrix::TranslationMatrix(Matrix other) :
      Matrix(other)
  {

  }

  TranslationMatrix::TranslationMatrix(MatrixValue_t x, MatrixValue_t y, MatrixValue_t z) :
      Matrix(4, 1)
  {
    (*this)[0][0] = x;
    (*this)[1][0] = y;
    (*this)[2][0] = z;
    (*this)[3][0] = 1;
  }

  TranslationMatrix::~TranslationMatrix()
  {

  }

  RotationMatrix::RotationMatrix() :
      Matrix(3, 3)
  {

  }

  RotationMatrix::RotationMatrix(Matrix other) :
      Matrix(other)
  {

  }

  RotationMatrix::~RotationMatrix()
  {

  }

  /*
   * Link Class Implementation
   */
  Link::Link() :
      theta_(0),
      alpha_(0),
      a_(0),
      d_(0),

      link_name_(""),
      servo_addr_(-1)
  {
    update_transformation_matrix();
  }

  Link::Link(MatrixValue_t thetaIn, MatrixValue_t alphaIn, MatrixValue_t aIn, MatrixValue_t dIn, std::string linkNameIn, int servoAddrIn) :
      theta_(thetaIn),
      alpha_(alphaIn),
      a_(aIn),
      d_(dIn),
      cos_alpha_(cos(alpha_)),
      sin_alpha_(sin(alpha_)),
      cos_theta_(cos(theta_)),
      sin_theta_(sin(theta_)),
      link_name_(linkNameIn),
      servo_addr_(servoAddrIn)
  {
    //std::cout << "Link <" << link_name_ << "> created" << std::endl;
    update_transformation_matrix();
  }

  const TransformationMatrix & Link::update_transformation_matrix(MatrixValue_t theta)
  {
    set_theta(theta);
    update_transformation_matrix();
    return transformation_matrix_;
  }

  const TransformationMatrix & Link::update_transformation_matrix()
  {

    //First column
    transformation_matrix_[0][0] = cos_theta_;
    transformation_matrix_[1][0] = sin_theta_;
    transformation_matrix_[2][0] = 0;
    transformation_matrix_[3][0] = 0;

    //Second column
    transformation_matrix_[0][1] = -sin_theta_ * cos_alpha_;
    transformation_matrix_[1][1] = cos_theta_ * cos_alpha_;
    transformation_matrix_[2][1] = sin_alpha_;
    transformation_matrix_[3][1] = 0;

    //Third column
    transformation_matrix_[0][2] = sin_theta_ * sin_alpha_;
    transformation_matrix_[1][2] = -cos_theta_ * sin_alpha_;
    transformation_matrix_[2][2] = cos_alpha_;
    transformation_matrix_[3][2] = 0;

    //Fourth column
    transformation_matrix_[0][3] = a_ * cos_theta_;
    transformation_matrix_[1][3] = a_ * sin_theta_;
    transformation_matrix_[2][3] = d_;
    transformation_matrix_[3][3] = 1;

    return get_transformation_matrix();
  }

  const TransformationMatrix & Link::get_transformation_matrix() const
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
  Limb::Limb(Body &body, Link &body_link, Link &coxa_link, Link &femur_link, Link &tibia_link, std::string name):
      body_(body),
      body_link_(body_link),
      coxa_link_(coxa_link),
      femur_link_(femur_link),
      tibia_link_(tibia_link),
      limb_name_(name)
  {
    //TODO: calculate safe initial position from body and link information. Do this with normal forward kinematics. Need transformation matrix for body.
    if(!InverseKinematic(ForwardKinematic()))
    {
      std::cout << limb_name_ << " has invalid initial link values!" << std::endl;
    }
  }

  bool Limb::InverseKinematic(MatrixValue_t x, MatrixValue_t y, MatrixValue_t z)
  {
    end_point_position_[0][0] = x;
    end_point_position_[1][0] = y;
    end_point_position_[2][0] = z;
    end_point_position_[3][0] = 1;
    return InverseKinematic(end_point_position_);
  }

  bool Limb::InverseKinematic(const TranslationMatrix &position_global)
  {
    //If coxa alpha is < 0 && > -180 femur and tibia joint rotation is reversed.
    MatrixValue_t femur_tibia_theta_correction = 1;
    if(coxa_link_.alpha() < 0 && coxa_link_.alpha() > -180)
    {
      femur_tibia_theta_correction = -1;
    }

    //Inverse transformation from global frame to body link frame (coxa joint rotates around z-axis of body link frame).
    TranslationMatrix position_body_link = body_link_.get_transformation_matrix().InverseTransformation() * body_.get_transformation_matrix().InverseTransformation() * position_global;

    //Length from body link to tibia link (End point). Pytagoras -> sqrt(x^2, y^3)
    MatrixValue_t a = sqrtf(position_body_link[0][0] * position_body_link[0][0] + position_body_link[1][0] * position_body_link[1][0]);
    //std::cout << "a: " << a << std::endl;

    //Sanity check a
    if(a < 0 || a > coxa_link_.a() + femur_link_.a() + tibia_link_.a())
    {
      return false;
    }

    //Length from coxa link frame to tibia link frame. Pytagors -> sqrt()
    MatrixValue_t b = sqrtf((a - coxa_link_.a()) * (a - coxa_link_.a()) + position_body_link[2][0] * position_body_link[2][0]);
    //std::cout << "b: " << b << std::endl;

    //theta3 = math.acos((b ** 2 - self.link3.a ** 2 - self.link2.a ** 2) / (-2 * self.link3.a * self.link2.a)) * (180 / math.pi) - 180

    MatrixValue_t tibia_theta = femur_tibia_theta_correction * (M_PI - acos((b * b - tibia_link_.a() * tibia_link_.a() - femur_link_.a() * femur_link_.a()) / (- 2 * tibia_link_.a() * femur_link_.a())));
    //std::cout << "tibia_theta: " << tibia_theta << std::endl;

    MatrixValue_t ika1 = atan2(position_body_link[2][0], a - coxa_link_.a());
    //std::cout << "ika1: " << ika1 << std::endl;

    MatrixValue_t ika2 = acos((-tibia_link_.a() * tibia_link_.a() + femur_link_.a() * femur_link_.a() + b * b) / (2 * femur_link_.a() * b));
    //std::cout << "ika2: " << ika2 << std::endl;
    //IKA2 = math.acos((-self.link3.a ** 2 + self.link2.a ** 2 + b ** 2) / (2 * self.link2.a * b))

    MatrixValue_t femur_theta = femur_tibia_theta_correction * (ika1 + ika2);
    //std::cout << "femur_theta: " << femur_theta << std::endl;

    //Calculate Coxa theta
    MatrixValue_t coxa_theta;
    //If end point is exactly below coxa rotation axis
    if(position_body_link[0][0] == 0)
    {
      coxa_theta = coxa_link_.theta();
    }
    else if(position_body_link[0][0] < 0)
    {
      coxa_theta = M_PI + atan2(position_body_link[1][0], position_body_link[0][0]);
    }
    else
    {
      coxa_theta = atan2(position_body_link[1][0], position_body_link[0][0]);
    }

    //std::cout << "coxa_theta: " << coxa_theta << std::endl;

    //Set angles on joints.
    coxa_link_.update_transformation_matrix(coxa_theta);
    femur_link_.update_transformation_matrix(femur_theta);
    tibia_link_.update_transformation_matrix(tibia_theta);

    return true;
  }

  TranslationMatrix Limb::ForwardKinematic() const
  {

    TranslationMatrix end_point_position_matrix = TranslationMatrix(
        body_.get_transformation_matrix()
            * body_link_.get_transformation_matrix()
            * coxa_link_.get_transformation_matrix()
            * femur_link_.get_transformation_matrix()
            * tibia_link_.get_transformation_matrix()
            * TranslationMatrix(0, 0, 0));
    return end_point_position_matrix;
  }

  Limb::~Limb()
  {

  }

  /*
   * Body Implementation
   */
  Body::Body() :
      transformation_matrix_(TransformationMatrix()),
      roll_(0),
      pitch_(0),
      yaw_(0),
      cos_roll_(cos(roll_)),
      cos_pitch_(cos(pitch_)),
      cos_yaw_(cos(yaw_)),
      sin_roll_(sin(roll_)),
      sin_pitch_(sin(pitch_)),
      sin_yaw_(sin(yaw_)),
      x_(0),
      y_(0),
      z_(0)
  {
    update_transformation_matrix();
  }

  Body::~Body()
  {

  }

  const TransformationMatrix & Body::update_transformation_matrix(MatrixValue_t roll, MatrixValue_t pitch, MatrixValue_t yaw, MatrixValue_t x, MatrixValue_t y, MatrixValue_t z)
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

    return update_transformation_matrix();
  }

  const TransformationMatrix & Body::update_transformation_matrix()
  {

    //First Column
    transformation_matrix_[0][0] = cos_yaw_ * cos_pitch_;
    transformation_matrix_[1][0] = sin_yaw_ * cos_pitch_;
    transformation_matrix_[2][0] = -sin_pitch_;
    transformation_matrix_[3][0] = 0;

    //Second Column
    transformation_matrix_[0][1] = -cos_roll_ * sin_yaw_ + cos_yaw_ * sin_roll_ * sin_pitch_;
    transformation_matrix_[1][1] = cos_roll_ * cos_yaw_ + sin_roll_ * sin_pitch_ * sin_yaw_;
    transformation_matrix_[2][1] = cos_pitch_ * sin_roll_;
    transformation_matrix_[3][1] = 0;

    //Third Column
    transformation_matrix_[0][2] = sin_roll_ * sin_yaw_ + cos_roll_ * cos_yaw_ * sin_pitch_;
    transformation_matrix_[1][2] = -cos_yaw_ * sin_roll_ + cos_roll_ * sin_pitch_ * sin_yaw_;
    transformation_matrix_[2][2] = cos_roll_ * cos_pitch_;
    transformation_matrix_[3][2] = 0;

    //Fourth Column
    transformation_matrix_[0][3] = x_;
    transformation_matrix_[1][3] = y_;
    transformation_matrix_[2][3] = z_;
    transformation_matrix_[3][3] = 1;

    return transformation_matrix_;
  }

  const TransformationMatrix & Body::get_transformation_matrix() const
  {
    return transformation_matrix_;
  }

}
