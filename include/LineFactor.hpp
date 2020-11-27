#ifndef LINEFACTOR_HPP
#define LINEFACTOR_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/numericalDerivative.h>
#include <iostream>
#include <gtsam/base/Matrix.h>

// using namespace Project;
namespace Project {

// Point2 = (r, theta)
class LineFactor: public gtsam::NoiseModelFactor3<gtsam::Pose2, gtsam::Pose2, gtsam::Point2> {
  double rho_, theta_;

  public:
    // constructor
    LineFactor(
      const gtsam::Key ref_pose_key, const gtsam::Key new_pose_key, const gtsam::Key landmark_key,
      const double rho, const double theta, gtsam::SharedNoiseModel& model
    ):
    gtsam::NoiseModelFactor3<gtsam::Pose2, gtsam::Pose2, gtsam::Point2>(model, ref_pose_key, new_pose_key, landmark_key),
    rho_(rho), theta_(theta) {}

    // destructor
    virtual ~LineFactor() {}

    // error function
    gtsam::Vector lineError(const gtsam::Pose2& ref_pose, const gtsam::Pose2& new_pose, const gtsam::Point2& landmark) const {
      Pose ref_pose_narnia = Pose2_to_Pose(ref_pose);
      Pose new_pose_narnia = Pose2_to_Pose(new_pose);
      Line landmark_narnia = Point2_to_Line(landmark, ref_pose_narnia);
      Line new_landmark_narnia = landmark_narnia.convert_coords(new_pose_narnia);
      return gtsam::Vector2(rho_ - new_landmark_narnia.r, wrapAng(theta_ - new_landmark_narnia.th));
    }

    // evaluate error
    gtsam::Vector evaluateError(const gtsam::Pose2& pose1, const gtsam::Pose2& pose2, const gtsam::Point2& line,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none,
      boost::optional<gtsam::Matrix&> H3 = boost::none
    ) const override {
      if(H1) (*H1) = gtsam::numericalDerivative11<gtsam::Vector,gtsam::Pose2>(boost::bind(&LineFactor::lineError, this, _1, pose2, line), pose1);
      if(H2) (*H2) = gtsam::numericalDerivative11<gtsam::Vector,gtsam::Pose2>(boost::bind(&LineFactor::lineError, this, pose1, _1, line), pose2);
      if(H3) (*H3) = gtsam::numericalDerivative11<gtsam::Vector,gtsam::Point2>(boost::bind(&LineFactor::lineError, this, pose1, pose2, _1), line);
      // if(H1){
      //   gtsam::Matrix& m=*H1;
      //   gtsam::print(m,"H1:",std::cout);
      // }
      // if(H2){
      //   gtsam::Matrix& m=*H2;
      //   gtsam::print(m,"H2:",std::cout);
      // }
      // if(H3){
      //   gtsam::Matrix& m=*H3;
      //   gtsam::print(m,"H3:",std::cout);
      // }
      return lineError(pose1, pose2, line);
    }

    // access methods for debugging
    // print function
};
} // namespace Project

#endif 