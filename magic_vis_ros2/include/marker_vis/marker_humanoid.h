#ifndef MAGIC_VIS_MARKER_HUMANOID_H_
#define MAGIC_VIS_MARKER_HUMANOID_H_

#include <model/model_humanoid.h>
#include <marker_vis/marker_base.h>

#include <Goldfarb_Optimizer/QuadProg++.hh>

#define CONTACT_MU 0.5

namespace magic {

inline static Eigen::Matrix3d skew(const Eigen::Vector3d& vec) {
  Eigen::Matrix3d res;
  res <<      0., -vec[2],  vec[1],
          vec[2],      0., -vec[0],
         -vec[1],  vec[0],      0.;

  return res;
}

struct HumanState {
  // handcrafted assignment
  Eigen::VectorXd q;
  Eigen::VectorXd dq;

  Eigen::Vector2d contact;
  Eigen::Matrix<double, 6, 2> wrench; // 3 force + 3 torque
  Eigen::Matrix<double, 6, 2> wrench_des;

  // obtained with calculation
  Eigen::Matrix<double, 3, 2> foot;
  Eigen::Matrix<double, 3, 8> foot_corner;
  Eigen::Matrix<double, 3, 8> wrench_decomp;
  Eigen::Matrix<double, 3, 8> wrench_des_decomp;

  void init(int dof) {
    q = Eigen::VectorXd::Zero(dof+1);
    dq = Eigen::VectorXd::Zero(dof);

    wrench.setZero();
    wrench_des.setZero();
  }
};

struct DataMarkerHumanoid
{
  HumanState cur_state;
  std::vector<HumanState> mpc_predict;

  Eigen::Vector3d support_center;
  std::vector<Eigen::Vector3d> support_vert;
};

class MarkerHumanoid : public MarkerBase {
public:
  using Mat34 = Eigen::Matrix<double, 3, 4>;
  using Wrench = Eigen::Matrix<double, 6, 1>;

public:
  /**
   * @brief  Builds an uninitialized visualizer.
   */
  MarkerHumanoid(std::string urdf);
  ~MarkerHumanoid();

  /**
   * @brief  Do extra calculation with pinocchio library.
   */
  void DataHandle(bool wbc_fsm);

  /**
   * @brief  Publish the RVIZ tf from the defined data structure.
   */
  virtual void BuildRobotTF() const override;

  /**
   * @brief  Constructs the RVIZ markers from the defined data structure.
   * @return The array of RVIZ markers to be published.
   */
  MarkerArray BuildRobotState() const override;

  DataMarkerHumanoid data;
  ModelHumanoid *pino_model;

private:
  void SingleStateHandle(HumanState& state, std::vector<bool> cal_flag = std::vector<bool>{true, false, false});
  Mat34 WrenchDecompose(Wrench wrench, Vector3d foot, Mat34 corner);

  MarkerVec CreateSupport(const Vector3d& pos, const EEPos& pos_W) const;
  MarkerVec CreateFootstepElement(const Mat34 corner, const bool vis, const bool vis_cone) const;
  MarkerVec CreateFootstepForce(const Mat34 corner, const Mat34 force, const std::string name, const bool vis) const;
  MarkerVec CreateMpcTrajectories() const;

  Trajectory base, footl, footr;
  std::vector<std::string> foot_name = {"FOOT_L", "FOOT_R"};
  std::vector<std::string> foot_corner_name = {
                                          "FOOT_L_FR", "FOOT_L_FL", "FOOT_L_RL", "FOOT_L_RR", 
                                          "FOOT_R_FR", "FOOT_R_FL", "FOOT_R_RL", "FOOT_R_RR"};

  /*
    The problem is in the form:

    min 0.5 * x G x + g0 x
    s.t.
      CE^T x + ce0 = 0
      CI^T x + ci0 >= 0
  */
  GolDIdnani::GVect<double> z;
  // Cost
  GolDIdnani::GMatr<double> G;
  GolDIdnani::GVect<double> g0;

  // Equality
  GolDIdnani::GMatr<double> CE;
  GolDIdnani::GVect<double> ce0;

  // Inequality
  GolDIdnani::GMatr<double> CI;
  GolDIdnani::GVect<double> ci0;
};

} /* namespace magic */

#endif /* MAGIC_VIS_MARKER_HUMANOID_H_ */
