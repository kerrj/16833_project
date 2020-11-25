#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/base/numericalDerivative.h>
#include <iostream>

#include "util.hpp"

// Point2 = (r, theta)
class LineFactor: public NoiseModelFactor2<Pose2, Pose2, Point2> {

  protected:
    double rho_;
    double theta_;


  public:
    // default constructor
    LineFactor(rho_(0.0), theta_(0.0));
    // constructor
    LineFactor(const Key ref_pose_key, const Key new_pose_key, const Key landmark_key, const rho, const theta) {
      Base(ref_pose_key, new_pose_key, landmark_key);
      this.rho_ = rho;
      this.theta_ = theta;
    }

    // destructor
    virtual ~LineFactor() {}



    // print method
    // error function
    Vector lineError(const Pose2& ref_pose, const Pose2& new_pose, const Point2& landmark) {
      ref_pose_narnia = Pose2_to_Pose(ref_pose);
      new_pose_narnia = Pose2_to_Pose(new_pose);
      landmark_narnia = Line2_to_Line(landmark);
      new_landmark_narnia = landmark_narnia.convert_coords(new_pose_narnia);

      return Point2 error(this.rho_ - new_landmark_narnia.r, this.theta_ - new_landmark_narnia.theta);
    }

    // evaluate error
    Vector evaluateError(const Pose2& pose1, const Pose2& pose2, const Point2& line,
                    boost::optional<Matrix&> H1=boost::none,
                    boost::optional<Matrix&> H2=boost::none,
                    boost::optional<Matrix&> H3=boost::none,
                    ) const {

      if(H1) (*H1) = numericalDerivative11<Vector,Pose2>(boost::bind(&LineFactor::lineError, this, _1, pose2, line), pose1);
      if(H2) (*H2) = numericalDerivative11<Vector,Pose2>(boost::bind(&LineFactor::lineError, this, pose1, _1, line), pose2);
      if(H3) (*H3) = numericalDerivative11<Vector,Point2>(boost::bind(&LineFactor::lineError, this, pose1, pose2, _1), line);

      return lineError(pose1, pose2, line);
    }
    // access methods for debugging
}
