#ifndef MAGIC_VIS_MODEL_HUMANOID_H_
#define MAGIC_VIS_MODEL_HUMANOID_H_

#include <string>
#include <memory>
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include <pinocchio/fwd.hpp>
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

namespace magic {

class ModelHumanoid {
public:
  using Model          = pinocchio::Model;
  using Data           = pinocchio::Data;
  using Motion         = pinocchio::Motion;
  using Frame          = pinocchio::Frame;
  using SE3            = pinocchio::SE3;
  using ReferenceFrame = pinocchio::ReferenceFrame;

  using VectorXd       = Eigen::VectorXd;
  using MatrixXd       = Eigen::MatrixXd;

public:
  ModelHumanoid(std::string urdf_name);

  void updateKinematics(VectorXd q);
  void updateKinematics(VectorXd q, VectorXd dq);

  SE3 framePosition(std::string frame_name);
  SE3 framePosition(Model::FrameIndex index);

  Motion frameVelocity(std::string frame_name, ReferenceFrame rf);
  Motion frameVelocity(Model::FrameIndex index, ReferenceFrame rf);

  MatrixXd frameJacobian(std::string frame_name, ReferenceFrame rf);
  MatrixXd frameJacobian(Model::FrameIndex index, ReferenceFrame rf);

  const Model& getModel() { return *model_ptr_; }
  Data& getData() { return *data_ptr_; }

private:
  std::shared_ptr<const Model> model_ptr_;
  std::unique_ptr<Data> data_ptr_;

};

} /* namespace magic */

#endif /* MAGIC_VIS_MODEL_HUMANOID_H_ */