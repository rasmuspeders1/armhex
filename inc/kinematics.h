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
#include <numeric>

namespace kinematics
{

  typedef float MatrixValue_t;
  typedef std::vector<std::vector<MatrixValue_t> > BasicMatrix_t;
  class Matrix
  {
    public:
      Matrix(size_t numberOfRows, size_t numberOfColumns)
      {
        //Initialize member.
        rows_ = numberOfRows;
        columns_ = numberOfColumns;
        data.resize(numberOfRows,
            std::vector<MatrixValue_t>(numberOfColumns, 0.0));
      }

      virtual ~Matrix()
      {

      }

      void Print() const;

      Matrix Transpose() const
      {
        //init transpose matrix
        Matrix transpose = Matrix(columns_, rows_);
        for (unsigned int i = 0; i < rows_; i++)
        {
          for (unsigned int j = 0; j < columns_; j++)
          {
            transpose[j][i] = (*this)[i][j];
          }
        }
        return transpose;
      }

      /**
       * operator* calculates dot product of two matrices
       */
      Matrix operator*(const Matrix &other) const
      {

        if(columns_ != other.rows_)
        {
          //TODO: Throw exception.
          return Matrix(0, 0);
        }

        //initialize output matrix
        Matrix output_matrix = Matrix(rows_, other.columns_);

        unsigned int row;
        unsigned int col;
        unsigned int other_col;

        for (row = 0; row < rows_; row++)
        {
          for (other_col = 0; other_col < other.columns_; other_col++)
          {
            MatrixValue_t product_sum = 0;
            for (col = 0; col < columns_; col++)
            {
              product_sum += (*this)[row][col] * other[col][other_col];
            }
            output_matrix[row][other_col] = product_sum;
          }
        }

        return output_matrix;
      }

      std::vector<MatrixValue_t> & operator[](size_t n)
      {
        return data[n];
      }

      const std::vector<MatrixValue_t>& operator[](size_t n) const
          {
        return data[n];
      }

    private:
      //2D vector of actual data
      BasicMatrix_t data;
      unsigned int rows_;
      unsigned int columns_;
  };

  /**
   * Translation Matrix.
   */
  class TranslationMatrix : public Matrix
  {
    public:
      TranslationMatrix();
      TranslationMatrix(Matrix other);
      TranslationMatrix(MatrixValue_t x, MatrixValue_t y, MatrixValue_t z);
      virtual ~TranslationMatrix();

    private:

  };

  /**
   * Rotation Matrix
   */
  class RotationMatrix : public Matrix
  {
    public:
      RotationMatrix();
      RotationMatrix(Matrix other);
      virtual ~RotationMatrix();
      private:

  };

  /**
   * Transformation Matrix
   */
  class TransformationMatrix : public Matrix
  {
    public:
      TransformationMatrix();
      TransformationMatrix(Matrix other);
      virtual ~TransformationMatrix();
      TranslationMatrix Translation() const;
      TransformationMatrix InverseTransformation() const;
      RotationMatrix Rotation() const;
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
      Link(MatrixValue_t theta, MatrixValue_t alpha, MatrixValue_t a, MatrixValue_t d, std::string link_name, int servo_addr);
      virtual ~Link();

      MatrixValue_t theta()
      {
        return theta_;
      }

      MatrixValue_t alpha()
      {
        return alpha_;
      }

      MatrixValue_t a()
      {
        return a_;
      }

      MatrixValue_t d()
      {
        return d_;
      }

      const TransformationMatrix& get_transformation_matrix() const;
      const TransformationMatrix& update_transformation_matrix(MatrixValue_t theta);
      const TransformationMatrix& update_transformation_matrix();

      int get_servo_addr()
      {
        return servo_addr_;
      }

    private:
      MatrixValue_t theta_;
      MatrixValue_t alpha_;
      MatrixValue_t a_;
      MatrixValue_t d_;
      MatrixValue_t cos_alpha_;
      MatrixValue_t sin_alpha_;
      MatrixValue_t cos_theta_;
      MatrixValue_t sin_theta_;

      std::string link_name_;

      //Address of the servo assigned to this link on the servo controller. Set to negative if this link has no servo.
      int servo_addr_;

      TransformationMatrix transformation_matrix_;

      void set_theta(MatrixValue_t theta)
      {
        theta_ = theta;
        sin_theta_ = sin(theta_);
        cos_theta_ = cos(theta_);
      }

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
      const TransformationMatrix & get_transformation_matrix() const;

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
      const TransformationMatrix & update_transformation_matrix(MatrixValue_t roll, MatrixValue_t pitch, MatrixValue_t yaw, MatrixValue_t x, MatrixValue_t y, MatrixValue_t z);

      const TransformationMatrix & update_transformation_matrix();

      MatrixValue_t roll() const {return roll_;}
      MatrixValue_t pitch() const {return pitch_;}
      MatrixValue_t yaw() const {return yaw_;}

      MatrixValue_t x() const {return x_;}
      MatrixValue_t y() const {return y_;}
      MatrixValue_t z() const {return z_;}

    private:

      TransformationMatrix transformation_matrix_; //Transformation matrix from body frame to global frame

      MatrixValue_t roll_; //Roll of the body frame. Rotation around the body frame y-axis.
      MatrixValue_t pitch_; //Pitch of the body frame. Rotation around the body frame x-axis.
      MatrixValue_t yaw_; //Yaw of the body frame. Rotation around the body frame z-axis.
      MatrixValue_t cos_roll_;
      MatrixValue_t cos_pitch_;
      MatrixValue_t cos_yaw_;
      MatrixValue_t sin_roll_;
      MatrixValue_t sin_pitch_;
      MatrixValue_t sin_yaw_;
      MatrixValue_t x_; //Position of the body frame along the x-axis of the global frame.
      MatrixValue_t y_; //Position of the body frame along the y-axis of the global frame.
      MatrixValue_t z_; //Position of the body frame along the z-axis of the global frame.

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
      Limb(Body &body, Link &body_link, Link &coxa_link, Link &femur_link, Link &tibia_link, std::string name);
      virtual ~Limb();

      /**
       * Calculates the theta anlges of all links based on the inpu position coordinates in the global frame.
       * returns false if inverse kinematic calculation was not possible. Probably because unreachable x,y,z coordinates.
       * @param x
       * @param y
       * @param z
       * return result
       */
      bool InverseKinematic(MatrixValue_t x, MatrixValue_t y, MatrixValue_t z);
      bool InverseKinematic(const TranslationMatrix &position_global);

      /**
       * Calculates the current position of t he limb end point based on the link angles, body position and orientation.
       * Returns a translation matrix
       * No sanity checking performed!
       */
      TranslationMatrix ForwardKinematic() const;

    private:

      TranslationMatrix end_point_position_; //Translation matrix from limb end point position frame to global frame.

      Body &body_; //The body object to which this limb is connected

      Link &body_link_; //The link connected to the origin of the body frame. This is fixed and cannot move.
      Link &coxa_link_; //The coxa link is connected to the body links and the femur link. Rotates around a vertical axis in the body frame.
      Link &femur_link_; //The femur link is connected to the coxa link and the tibia link. Rotates a round a horizontal axis in the body frame.
      Link &tibia_link_; //The tibia link is connected to the femur link and its origin is the end effector position. Rotates a round a horizontal axis in the body frame.
      std::string limb_name_;
  };

}

#endif /* KINEMATICS_H_ */
