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

namespace kinematics
{

  typedef float Value_t;
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
          data.resize(numberOfRows, std::vector<Value_t>(numberOfColumns, 0.0));
        }
        virtual ~Matrix()
        {

        }
        parentType Transpose()
        {
          return parentType(this);
        }

        std::vector<Value_t> & operator[](size_t n)
        {
          return data[n];
        }
        const std::vector<Value_t>& operator[](size_t n) const
        {
          return data[n];
        }

      private:
        //2D vector of actual data
        std::vector<std::vector<Value_t> > data;
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
      Link(Value_t theta, Value_t alpha, Value_t a, Value_t d,
          const std::string &link_name, int servo_addr);
      virtual ~Link();

      void set_theta(Value_t val)
      {
        theta_ = val / (180.0 / M_1_PI);
      }
      const TransformationMatrix& getTransformationMatrix();
      const TransformationMatrix& getTransformationMatrix(Value_t theta);
      int getServoAddr(){return servo_addr_;}

    private:
      Value_t theta_;
      Value_t alpha_;
      Value_t a_;
      Value_t d_;
      Value_t cos_alpha_;
      Value_t sin_alpha_;
      Value_t cos_theta_;
      Value_t sin_theta_;

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
    private:

  };

  class Limb
  {
    public:
      Limb();
      Limb(Body body, Link body_link, Link coxa_link, Link femur_link, Link tibia_link);
      virtual ~Limb();

      bool InverseKinematic(Value_t x, Value_t y, Value_t z);

    private:

      TranslationMatrix end_point_position_;

      Body body_;

      Link body_link_;
      Link coxa_link_;
      Link femur_link_;
      Link tibia_link_;

  };


}

#endif /* KINEMATICS_H_ */
