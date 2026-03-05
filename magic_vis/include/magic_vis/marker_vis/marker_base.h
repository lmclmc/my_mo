#ifndef MAGIC_VIS_MARKER_BASE_H_
#define MAGIC_VIS_MARKER_BASE_H_

#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>

#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <magic_vis/marker_vis/rviz_colors.h>

namespace magic {

inline static geometry_msgs::Point v3dToGeoPoint(const Eigen::Vector3d& pos) {
  geometry_msgs::Point res;
  
  res.x = pos[0];
  res.y = pos[1];
  res.z = pos[2];

  return res;
}

inline static geometry_msgs::Vector3 v3dToGeoV3(const Eigen::Vector3d& pos) {
  geometry_msgs::Vector3 res;
  
  res.x = pos[0];
  res.y = pos[1];
  res.z = pos[2];

  return res;
}

inline static geometry_msgs::Quaternion qautToGeoQuat(Eigen::Quaterniond quat) {
  geometry_msgs::Quaternion res;

  res.x = quat.x();
  res.y = quat.y();
  res.z = quat.z();
  res.w = quat.w();

  return res;
}

struct DataMarkerBase
{
  Eigen::Vector3d pos;
  Eigen::Quaterniond ori;

  Eigen::Vector3d com;
  
  DataMarkerBase() {
    pos.setZero();
    com.setZero();

    ori.x() = 0.;
    ori.y() = 0.;
    ori.z() = 0.;
    ori.w() = 1.;
  }
};

class MarkerBase {
public:
  using Vector3d        = Eigen::Vector3d;

  using Marker          = visualization_msgs::Marker;
  using MarkerVec       = std::vector<Marker>;
  using MarkerArray     = visualization_msgs::MarkerArray;

  using ContactState    = std::vector<bool>;
  using EEPos           = std::vector<Vector3d>;
  using EEForces        = std::vector<Vector3d>;
  using Trajectory      = std::vector<Vector3d>;

public:
  /**
   * @brief  Builds an uninitialized visualizer.
   */
  MarkerBase() = default;
  virtual ~MarkerBase () = default;

  /**
   * @brief  Publish the RVIZ tf from the defined data structure.
   */
  virtual void BuildRobotTF() const;

  /**
   * @brief  Constructs the RVIZ markers from the defined data structure.
   * @return The array of RVIZ markers to be published.
   */
  virtual MarkerArray BuildRobotState() const;

  DataMarkerBase data;

protected:
  void publishTF(Vector3d& pos, Eigen::Quaterniond ori, std::string name);

  Marker    CreateBasePose(const Vector3d& pos, Eigen::Quaterniond ori) const;
  Marker    CreateCoM(const Vector3d& pos) const;
  Marker    CreateCoMProjection(const Vector3d& pos) const;

  MarkerVec CreateEEPositions(const EEPos& pos_W) const;
  MarkerVec CreateEEForces(const EEForces& f_W, const EEPos& pos_W, const std::string name, const bool vis) const;
  MarkerVec CreateFrictionCones(const EEPos& ee_pos, const ContactState& contact_state, double friction_coeff) const;
  Marker    CreateFrictionCone(const Vector3d& pos_W, const Vector3d& terrain_normal, double friction_coeff) const;
  Marker    CreatePolygonArea(const EEPos& ee_pos) const;
  Marker    CreateSupportMarginPoint(const Vector3d& pos) const;
  Marker    CreateGravityForce(const double mass, const Vector3d& base_pos) const;
  
  Marker    CreateForceArrow(const Vector3d& f, const Vector3d& pos) const;
  Marker    CreateTrajectory(const Trajectory& traj) const;
  Marker    CreateSphere(const Vector3d& pos, double diameter = 0.03) const;
  Marker    CreateBox(const Vector3d& pos, Eigen::Quaterniond ori, const Vector3d& edge_length) const;
  
  const std::string frame_id_ = "world";

  tf::TransformBroadcaster tf_broadcaster_;
};

} /* namespace magic */

#endif /* MAGIC_VIS_MARKER_BASE_H_ */
