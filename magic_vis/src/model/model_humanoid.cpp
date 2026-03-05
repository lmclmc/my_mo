#include <pinocchio/fwd.hpp>
#include <magic_vis/model/model_humanoid.h>

namespace magic {

ModelHumanoid::ModelHumanoid(std::string urdf_name)
{
  // Load the urdf model
  Model model;
  pinocchio::urdf::buildModel(urdf_name, pinocchio::JointModelFreeFlyer(), model);  // floating base model

  SE3 foot_place(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., -0.043));
  SE3 foot_fr_place(Eigen::Matrix3d::Identity(), Eigen::Vector3d( 0.16, -0.05, -0.043));
  SE3 foot_fl_place(Eigen::Matrix3d::Identity(), Eigen::Vector3d( 0.16,  0.05, -0.043));
  SE3 foot_rl_place(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-0.08,  0.05, -0.043));
  SE3 foot_rr_place(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-0.08, -0.05, -0.043));

  Frame footl = Frame("FOOT_L", model.getJointId("JOINT_ANKLE_ROLL_L"), model.getFrameId("JOINT_ANKLE_ROLL_L"), foot_place, pinocchio::FrameType::OP_FRAME);
  Frame footl_fr = Frame("FOOT_L_FR", model.getJointId("JOINT_ANKLE_ROLL_L"), model.getFrameId("JOINT_ANKLE_ROLL_L"), foot_fr_place, pinocchio::FrameType::OP_FRAME);
  Frame footl_fl = Frame("FOOT_L_FL", model.getJointId("JOINT_ANKLE_ROLL_L"), model.getFrameId("JOINT_ANKLE_ROLL_L"), foot_fl_place, pinocchio::FrameType::OP_FRAME);
  Frame footl_rl = Frame("FOOT_L_RL", model.getJointId("JOINT_ANKLE_ROLL_L"), model.getFrameId("JOINT_ANKLE_ROLL_L"), foot_rl_place, pinocchio::FrameType::OP_FRAME);
  Frame footl_rr = Frame("FOOT_L_RR", model.getJointId("JOINT_ANKLE_ROLL_L"), model.getFrameId("JOINT_ANKLE_ROLL_L"), foot_rr_place, pinocchio::FrameType::OP_FRAME);

  Frame footr = Frame("FOOT_R", model.getJointId("JOINT_ANKLE_ROLL_R"), model.getFrameId("JOINT_ANKLE_ROLL_R"), foot_place, pinocchio::FrameType::OP_FRAME);
  Frame footr_fr = Frame("FOOT_R_FR", model.getJointId("JOINT_ANKLE_ROLL_R"), model.getFrameId("JOINT_ANKLE_ROLL_R"), foot_fr_place, pinocchio::FrameType::OP_FRAME);
  Frame footr_fl = Frame("FOOT_R_FL", model.getJointId("JOINT_ANKLE_ROLL_R"), model.getFrameId("JOINT_ANKLE_ROLL_R"), foot_fl_place, pinocchio::FrameType::OP_FRAME);
  Frame footr_rl = Frame("FOOT_R_RL", model.getJointId("JOINT_ANKLE_ROLL_R"), model.getFrameId("JOINT_ANKLE_ROLL_R"), foot_rl_place, pinocchio::FrameType::OP_FRAME);
  Frame footr_rr = Frame("FOOT_R_RR", model.getJointId("JOINT_ANKLE_ROLL_R"), model.getFrameId("JOINT_ANKLE_ROLL_R"), foot_rr_place, pinocchio::FrameType::OP_FRAME);

  model.addFrame(footl, false);
  model.addFrame(footl_fr, false);
  model.addFrame(footl_fl, false);
  model.addFrame(footl_rl, false);
  model.addFrame(footl_rr, false);

  model.addFrame(footr, false);
  model.addFrame(footr_fr, false);
  model.addFrame(footr_fl, false);
  model.addFrame(footr_rl, false);
  model.addFrame(footr_rr, false);

  model_ptr_ = std::make_shared<const Model>(model);
  data_ptr_ = std::make_unique<Data>(*model_ptr_);
}

void
ModelHumanoid::updateKinematics(VectorXd q)
{
  VectorXd dq = VectorXd::Zero(model_ptr_->nv);
  updateKinematics(q, dq);
}

void 
ModelHumanoid::updateKinematics(VectorXd q, VectorXd dq)
{
  if (q.size()!=model_ptr_->nq) {
    std::printf("Bad input generalized coordinate dimension (input_size - nq = %ld), and %s\n", q.size()-model_ptr_->nq,
                (q.size()-model_ptr_->nq<0) ? "fullfill with 0" : "discard redundant data");
    q.resize(model_ptr_->nq);
  }

  if (fabs(q.segment(3, 4).norm() - 1) > 1e-3) {
    std::printf("Bad input generalized coordinate quaternion, auto normalizing...\n");
    std::printf("ori raw: x(%f), y(%f), z(%f), w(%f)\n", q[3], q[4], q[5], q[6]);

    if (q.segment(3, 4).norm()==0)
      q(6) = 1.;
    else
      q.segment(3, 4).normalized();

    std::printf("ori modified: x(%f), y(%f), z(%f), w(%f)\n", q[3], q[4], q[5], q[6]);
  }

  if(dq.size()!=model_ptr_->nv) {
    std::printf("Bad input generalized velocity dimension (input_size - nv = %ld), and %s\n", dq.size()-model_ptr_->nv,
                (dq.size()-model_ptr_->nv<0) ? "fullfill with 0" : "discard redundant data");
    dq.resize(model_ptr_->nv);
  }

  pinocchio::forwardKinematics(*model_ptr_, *data_ptr_, q, dq);
  pinocchio::updateFramePlacements(*model_ptr_, *data_ptr_);
  // pinocchio::computeJointJacobians(*model_ptr_, *data_ptr_);
}

ModelHumanoid::SE3 
ModelHumanoid::framePosition(std::string frame_name)
{
  return data_ptr_->oMf[model_ptr_->getFrameId(frame_name)];
}

ModelHumanoid::SE3 
ModelHumanoid::framePosition(Model::FrameIndex index)
{
  return data_ptr_->oMf[index];
}

ModelHumanoid::Motion
ModelHumanoid::frameVelocity(std::string frame_name, ReferenceFrame rf)
{
  return pinocchio::getFrameVelocity(*model_ptr_, *data_ptr_, model_ptr_->getFrameId(frame_name), rf);
}

ModelHumanoid::Motion
ModelHumanoid::frameVelocity(Model::FrameIndex index, ReferenceFrame rf)
{
  return pinocchio::getFrameVelocity(*model_ptr_, *data_ptr_, index, rf);
}

ModelHumanoid::MatrixXd
ModelHumanoid::frameJacobian(std::string frame_name, ReferenceFrame rf)
{
  MatrixXd J = MatrixXd::Zero(6, model_ptr_->nv);
  pinocchio::getFrameJacobian(*model_ptr_, *data_ptr_, model_ptr_->getFrameId(frame_name), rf, J);
  return J;
}

ModelHumanoid::MatrixXd
ModelHumanoid::frameJacobian(Model::FrameIndex index, ReferenceFrame rf)
{
  MatrixXd J = MatrixXd::Zero(6, model_ptr_->nv);
  pinocchio::getFrameJacobian(*model_ptr_, *data_ptr_, index, rf, J);
  return J;
}


} /* namespace magic */