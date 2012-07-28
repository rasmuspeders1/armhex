/*
 * kinematics.h
 *
 *  Created on: Jul 14, 2012
 *      Author: rasmus
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <string>
#include <vector>
#include <math.h>
#include <iostream>

namespace kinematics
{

  typedef float MatrixValue;
  /**
   * Template class to create specific Matrix classes
   */
  template<class parentType>
    class Matrix
    {
      public:
        Matrix(size_t numberOfRows, size_t numberOfColumns)
        {
          //Initialize member.
          data.resize(numberOfRows,
              std::vector<MatrixValue>(numberOfColumns, 0.0));
        }
        virtual ~Matrix()
        {

        }
        parentType Transpose()
        {
          return parentType(this);
        }

        std::vector<MatrixValue> & operator[](size_t n)
        {
          return data[n];
        }
        const std::vector<MatrixValue>& operator[](size_t n) const
        {
          return data[n];
        }

      private:
        //2D vector of actual data
        std::vector<std::vector<MatrixValue> > data;
    };

  /**
   * Translation Matrix.
   */
  class TranslationMatrix : public Matrix<TranslationMatrix>
  {
    public:
      TranslationMatrix();
      virtual ~TranslationMatrix();

    private:

  };

  /**
   * Rotation Matrix
   */
  class RotationMatrix : public Matrix<RotationMatrix>
  {
    public:
      RotationMatrix();
      virtual ~RotationMatrix();
    private:

  };

  /**
   * Transformation Matrix
   */
  class TransformationMatrix : public Matrix<TransformationMatrix>
  {
    public:
      TransformationMatrix();
      virtual ~TransformationMatrix();
      TranslationMatrix getTranslationMatrix();
      TransformationMatrix getInverseTransformationMatrix();
    private:

  };

  /**
   * Link class
   * represents a Denavit Hartenberg Style kinematic link.
   */
  class Link
  {
    public:
      Link();
      Link(MatrixValue theta, MatrixValue alpha, MatrixValue a, MatrixValue d, std::string link_name, int servo_addr);
      virtual ~Link();

      MatrixValue get_theta()
      {
        std::cout << link_name_ << " has theta = " << theta_ << std::endl;
        return theta_;
      }

      const TransformationMatrix& get_transformation_matrix();
      const TransformationMatrix& update_transformation_matrix(MatrixValue theta);
      int get_servo_addr()
      {
        return servo_addr_;
      }

    private:
      MatrixValue theta_;
      MatrixValue alpha_;
      MatrixValue a_;
      MatrixValue d_;
      MatrixValue cos_alpha_;
      MatrixValue sin_alpha_;
      MatrixValue cos_theta_;
      MatrixValue sin_theta_;

      std::string link_name_;

      //Address of the servo assigned to this link on the servo controller. Set to negative if this link has no servo.
      int servo_addr_;

      TransformationMatrix transformation_matrix_;

  };

  class Body
  {
    public:
      Body();
      virtual ~Body();

      /**
       * Method that returns the body transformation matrix from body frame to global frame.
       * uses the object current member values
       * @return transformation_matrix
       */
      const TransformationMatrix & get_transformation_matrix();

      /**
       * Method that updates the body transformation matrix from body frame to global frame.
       * Takes roll, pitch, yaw and x, y, z input updates its internal members and returns an uprated transformation matrix.
       * Returns a const reference for the transformation matrix
       * @param roll
       * @param pitch
       * @param yaw
       * @param x
       * @param y
       * @param z
       * @return transformation_matrix
       */
      const TransformationMatrix & update_transformation_matrix(MatrixValue roll, MatrixValue pitch, MatrixValue yaw, MatrixValue x, MatrixValue y, MatrixValue z);

    private:

      void update_transformation_matrix();

      TransformationMatrix transformation_matrix_; //Transformation matrix from body frame to global frame

      MatrixValue roll_; //Roll of the body frame. Rotation around the body frame y-axis.
      MatrixValue pitch_; //Pitch of the body frame. Rotation around the body frame x-axis.
      MatrixValue yaw_; //Yaw of the body frame. Rotation around the body frame z-axis.
      MatrixValue cos_roll_;
      MatrixValue cos_pitch_;
      MatrixValue cos_yaw_;
      MatrixValue sin_roll_;
      MatrixValue sin_pitch_;
      MatrixValue sin_yaw_;
      MatrixValue x_; //Position of the body frame along the x-axis of the global frame.
      MatrixValue y_; //Position of the body frame along the y-axis of the global frame.
      MatrixValue z_; //Position of the body frame along the z-axis of the global frame.

  };

  class Limb
  {
    public:
      /**
       * Limb constructor
       * Takes a const reference to the body it is connected to. This is needed to be able to get the body's positional and rotational information
       * Also takes the three links the limb is made up of.
       * @param body
       * @param body_link
       * @param coxa_link
       * @param femur_link
       * @param tibia_link
       */
      Limb(const Body &body, Link &body_link, Link &coxa_link, Link &femur_link, Link &tibia_link);
      virtual ~Limb();

      bool InverseKinematic(MatrixValue x, MatrixValue y, MatrixValue z);

    private:

      TranslationMatrix end_point_position_; //Translation matrix from limb end point position frame to global frame.

      Body body_; //The body object to which this limb is connected

      Link body_link_; //The link connected to the origin of the body frame. This is fixed and cannot move.
      Link coxa_link_; //The coxa link is connected to the body links and the femur link. Rotates around a vertical axis in the body frame.
      Link femur_link_; //The femur link is connected to the coxa link and the tibia link. Rotates a round a horizontal axis in the body frame.
      Link tibia_link_; //The tibia link is connected to the femur link and its origin is the end effector position. Rotates a round a horizontal axis in the body frame.

  };

}

#endif /* KINEMATICS_H_ */
