#include <magic_vis/marker_vis/marker_base.h>

namespace magic {

void 
MarkerBase::BuildRobotTF() const
{

}

MarkerBase::MarkerArray
MarkerBase::BuildRobotState() const
{
  MarkerArray msg;

  Marker base = CreateBasePose(data.pos, data.ori);
  msg.markers.push_back(base);

  Marker com = CreateCoM(data.com);
  msg.markers.push_back(com);

  Marker cpro = CreateCoMProjection(data.com);
  msg.markers.push_back(cpro);
  
  int id = 0;
  for (Marker& m : msg.markers) {
    m.header.frame_id = frame_id_;
    m.id = id++;
  }

  return msg;
}

MarkerBase::Marker    
MarkerBase::CreateCoM(const Vector3d& pos) const
{
  Marker m = CreateSphere(pos, 0.05);
  m.ns     = "com";
  m.color  = color.blue;

  return m;
}

MarkerBase::Marker    
MarkerBase::CreateCoMProjection(const Vector3d& pos) const
{
  Marker m = CreateSphere(Vector3d(pos[0], pos[1], 0.), 0.01);
  m.ns     = "com_gravity_projection";
  m.color  = color.blue;

  return m;
}

MarkerBase::Marker
MarkerBase::CreateBasePose(const Vector3d& pos, Eigen::Quaterniond ori) const
{
  Vector3d edge_length(0.05, 0.05, 0.02);
  Marker m = CreateBox(pos, ori, 3*edge_length);

  m.color = color.black;
  m.color.a = 0.8;

  m.ns = "base_pose";

  return m;
}

MarkerBase::MarkerVec
MarkerBase::CreateEEPositions(const EEPos& ee_pos) const
{
  MarkerVec vec;

  for (auto ee : ee_pos) {
    Marker m = CreateSphere(ee, 0.04);
    m.ns     = "endeffector_pos";
    m.color  = color.blue;

    vec.push_back(m);
  }

  return vec;
}

MarkerBase::Marker
MarkerBase::CreateSupportMarginPoint(const Vector3d& pos) const
{
  Marker m = CreateSphere(pos, 0.01);
  m.ns     = "middle_support_polygon";
  m.color  = color.red;

  return m;
}

MarkerBase::Marker
MarkerBase::CreateGravityForce(const double mass, const Vector3d& base_pos) const
{
  double g = 9.81;
  Marker m = CreateForceArrow(Eigen::Vector3d(0., 0., -mass*g), base_pos);
  m.color  = color.red;
  m.ns     = "gravity_force";

  return m;
}

MarkerBase::MarkerVec
MarkerBase::CreateEEForces(const EEForces& ee_forces, const EEPos& ee_pos, const std::string name, const bool vis) const
{
  MarkerVec vec;

  for (int i=0; i<ee_forces.size(); i++) {
    Vector3d p = ee_pos.at(i);
    Vector3d f = ee_forces.at(i);

    Marker m  = CreateForceArrow(f, p);
    m.color   = color.red;
    m.color.a = ((vis && (f.norm() > 1.)) ? 1. : 0.);
    m.ns      = name;
    vec.push_back(m);
  }

  return vec;
}

MarkerBase::MarkerVec
MarkerBase::CreateFrictionCones(const EEPos& ee_pos, const ContactState& contact_state, double mu) const
{
  MarkerVec vec;

  // only draw cones if terrain_msg and robot state correspond
  for (int i=0; i<ee_pos.size(); i++) {
    Marker m;
    Vector3d n(0., 0., 1.); // @todo 
    m         = CreateFrictionCone(ee_pos.at(i), n, mu);
    m.color   = color.yellow;
    m.color.a = (((mu > 1e-3) && contact_state.at(i))? 0.25 : 0.);
    vec.push_back(m);
  }

  return vec;
}

MarkerBase::Marker
MarkerBase::CreateFrictionCone(const Vector3d& pos, const Vector3d& normal, double mu) const
{
  Marker m;
  m.type = Marker::ARROW;
  m.ns   = "friction_cone";

  double cone_height = 0.1; // [m]

  m.scale.x = 0.; // [shaft diameter] hide arrow shaft to generate cone
  m.scale.y = 2. * cone_height * mu; // arrow-head diameter
  m.scale.z = cone_height;            // arrow head length

  auto start = v3dToGeoPoint(pos + normal);
  m.points.push_back(start);

  auto end = v3dToGeoPoint(pos);
  m.points.push_back(end);

  return m;
}

MarkerBase::Marker
MarkerBase::CreatePolygonArea(const EEPos& ee_pos) const
{
  Marker m;
  m.scale.x = 1.;
  m.scale.y = 1.;
  m.scale.z = 1.;
  m.type = Marker::TRIANGLE_LIST;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.position.x = 0.;
  m.pose.position.y = 0.;
  m.pose.position.z = 0.;
  m.pose.orientation.x = 0.;
  m.pose.orientation.y = 0.;
  m.pose.orientation.z = 0.;
  m.pose.orientation.w = 1.;

  if (ee_pos.size()>=4) {
    geometry_msgs::Point tr1, tr2, tr3;
    tr3 = v3dToGeoPoint(ee_pos.at(ee_pos.size()-1));
    
    for (int i=0; i<ee_pos.size()-2; i++) {
      tr1 = v3dToGeoPoint(ee_pos.at(i));
      tr2 = v3dToGeoPoint(ee_pos.at(i+1));
      m.points.push_back(tr1);
      m.points.push_back(tr2);
      m.points.push_back(tr3);
    }
  }
  else {
    m.color.a = 0.; // hide marker
    auto p = v3dToGeoPoint(Vector3d(0., 0., 0.));
    m.points.push_back(p);
    m.points.push_back(p);
    m.points.push_back(p);
  }

  return m;
}

MarkerBase::Marker
MarkerBase::CreateForceArrow(const Vector3d& force, const Vector3d& ee_pos) const
{
  Marker m;
  m.type = Marker::ARROW;

  m.scale.x = 0.01; // shaft diameter
  m.scale.y = 0.02; // arrow-head diameter
  m.scale.z = 0.06; // arrow-head length

  m.pose.position.x = 0.;
  m.pose.position.y = 0.;
  m.pose.position.z = 0.;
  m.pose.orientation.x = 0.;
  m.pose.orientation.y = 0.;
  m.pose.orientation.z = 0.;
  m.pose.orientation.w = 1.;

  auto start = v3dToGeoPoint(ee_pos);
  m.points.push_back(start);

  double force_scale = 300;
  Vector3d p_end = ee_pos + force/force_scale;
  auto end = v3dToGeoPoint(p_end);
  m.points.push_back(end);

  return m;
}

MarkerBase::Marker
MarkerBase::CreateTrajectory(const Trajectory& traj) const
{
  Marker m;
  
  m.type = Marker::LINE_STRIP;
  m.action = visualization_msgs::Marker::ADD;

  m.scale.x = 0.01;

  for (auto p : traj)
    m.points.push_back(v3dToGeoPoint(p));

  return m;
}

MarkerBase::Marker
MarkerBase::CreateSphere(const Vector3d& pos, double diameter) const
{
  Marker m;

  m.type = Marker::SPHERE;
  m.pose.position = v3dToGeoPoint(pos);
  m.pose.orientation.x = 0.;
  m.pose.orientation.y = 0.;
  m.pose.orientation.z = 0.;
  m.pose.orientation.w = 1.;
  m.scale.x = diameter;
  m.scale.y = diameter;
  m.scale.z = diameter;

  return m;
}

MarkerBase::Marker
MarkerBase::CreateBox(const Vector3d& pos, Eigen::Quaterniond ori, const Vector3d& edge_length) const
{
  Marker m;

  m.type = Marker::CUBE;
  m.pose.position    = v3dToGeoPoint(pos);
  m.pose.orientation = qautToGeoQuat(ori);
  m.scale            = v3dToGeoV3(edge_length);

  return m;
}

void 
MarkerBase::publishTF(Vector3d& pos, Eigen::Quaterniond ori, std::string name)
{
  geometry_msgs::TransformStamped tf_msg;

  tf_msg.header.stamp    = ::ros::Time::now();
  tf_msg.header.frame_id = "world";
  tf_msg.child_frame_id  = name;

  tf_msg.transform.translation.x = pos[0];
  tf_msg.transform.translation.y = pos[1];
  tf_msg.transform.translation.z = pos[2];

  tf_msg.transform.rotation.w = ori.w();
  tf_msg.transform.rotation.x = ori.x();
  tf_msg.transform.rotation.y = ori.y();
  tf_msg.transform.rotation.z = ori.z();

  tf_broadcaster_.sendTransform(tf_msg);
}

} /* namespace magic */
