#include <pinocchio/fwd.hpp>
#include <marker_vis/marker_base.h>
#include <marker_vis/marker_humanoid.h>

#include <eigen3/Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/quaternion.hpp>

#include <lcm_to_ros2/msg/state_estimator_lcmt.hpp>
#include <lcm_to_ros2/msg/leg_control_data_lcmt.hpp>
#include <lcm_to_ros2/msg/arm_control_data_lcmt.hpp>
#include <lcm_to_ros2/msg/waist_control_data_lcmt.hpp>
#include <lcm_to_ros2/msg/head_control_data_lcmt.hpp>
#include <lcm_to_ros2/msg/lcm_response_lcmt.hpp>
#include <lcm_to_ros2/msg/wbc_test_data_t.hpp>
#include <lcm_to_ros2/msg/trajectory_joint_lcmt.hpp>

static magic::DataMarkerBase     data_marker_base;
static magic::DataMarkerHumanoid data_marker_humanoid;
static bool f1 = false;
static bool f2 = false;
static bool f3 = false;
static bool f4 = false;
static bool f5 = false;
static int fsm = 0;

std::shared_ptr<rclcpp::Node> g_Node;

static void StateCallback(const lcm_to_ros2::msg::StateEstimatorLcmt::SharedPtr state_msg)
{
  f1 = true;

  data_marker_base.pos << state_msg->p[0], state_msg->p[1], state_msg->p[2];
  data_marker_base.com << state_msg->com_p[0], state_msg->com_p[1], state_msg->com_p[2];

  data_marker_base.ori.w() = state_msg->quat[0];
  data_marker_base.ori.x() = state_msg->quat[1];
  data_marker_base.ori.y() = state_msg->quat[2];
  data_marker_base.ori.z() = state_msg->quat[3];

  data_marker_humanoid.cur_state.q.head(7) << data_marker_base.pos, 
                                              state_msg->quat[1], state_msg->quat[2], state_msg->quat[3], state_msg->quat[0];

  data_marker_humanoid.cur_state.dq.head(6) << state_msg->v_body[0], state_msg->v_body[1], state_msg->v_body[2], 
                                               state_msg->omega_body[0], state_msg->omega_body[1], state_msg->omega_body[2];

  data_marker_humanoid.cur_state.contact << state_msg->contact_estimateby_leg[0], state_msg->contact_estimateby_leg[1];

  data_marker_humanoid.support_center << state_msg->support_center[0], state_msg->support_center[1], 0.;

  data_marker_humanoid.support_vert.clear();
  for(int i=0; i<state_msg->vertex_num; i++) {
    data_marker_humanoid.support_vert.push_back(
      Eigen::Vector3d(state_msg->polygon_vertex[3*i], state_msg->polygon_vertex[3*i+1], state_msg->polygon_vertex[3*i+2])
    );
  }
}

static void legStateCallback(const lcm_to_ros2::msg::LegControlDataLcmt::SharedPtr state_msg)
{
  //RCLCPP_INFO(g_Node->get_logger(), "leg_control_data publisher created");
  f2 = true;

  for (int i=0; i<12; i++) {
    data_marker_humanoid.cur_state.q[i+7] = state_msg->q[i];
    data_marker_humanoid.cur_state.dq[i+6] = state_msg->qd[i];
  }
}

static void waistStateCallback(const lcm_to_ros2::msg::WaistControlDataLcmt::SharedPtr state_msg)
{
  f3 = true;

  for (int i=0; i<2; i++) {
    data_marker_humanoid.cur_state.q[i+7+12] = state_msg->q[i];
    data_marker_humanoid.cur_state.dq[i+6+12] = state_msg->qd[i];
  }
}

static void headStateCallback(const lcm_to_ros2::msg::HeadControlDataLcmt::SharedPtr state_msg)
{
  f4 = true;

  for (int i=0; i<2; i++) {
    data_marker_humanoid.cur_state.q[i+7+12+2] = state_msg->q[i];
    data_marker_humanoid.cur_state.dq[i+6+12+2] = state_msg->qd[i];
  }
}

static void armStateCallback(const lcm_to_ros2::msg::ArmControlDataLcmt::SharedPtr state_msg)
{
  f5 = true;

  for (int i=0; i<14; i++) {
    data_marker_humanoid.cur_state.q[i+7+12+2+2] = state_msg->joint_cur_pos[i];
    data_marker_humanoid.cur_state.dq[i+6+12+2+2] = state_msg->joint_cur_vel[i];
  }
}

static void fsmCallback(const lcm_to_ros2::msg::LcmResponseLcmt::SharedPtr state_msg)
{
  fsm = state_msg->robot_fsm;
}

static void wbcWrenchCallback(const lcm_to_ros2::msg::WbcTestDataT::SharedPtr state_msg) {
  for (int i=0; i<2; i++) {
    for (int j=0; j<3; j++) {
      data_marker_humanoid.cur_state.wrench(j, i) = state_msg->fr[6*i + 3 + j];
      data_marker_humanoid.cur_state.wrench(j+3, i) = state_msg->fr[6*i + j];

      data_marker_humanoid.cur_state.wrench_des(j, i) = state_msg->fr_des[6*i + 3 + j];
      data_marker_humanoid.cur_state.wrench_des(j+3, i) = state_msg->fr_des[6*i + j];
    }
  }
}

static void jointTrajectoryCallback(const lcm_to_ros2::msg::TrajectoryJointLcmt::SharedPtr state_msg) {
  data_marker_humanoid.mpc_predict.clear();
  magic::HumanState ini_state = data_marker_humanoid.cur_state;
  for (int i=0; i<10; i++) {
    ini_state.q[0] = state_msg->body_x[i];
    ini_state.q[1] = state_msg->body_y[i];
    ini_state.q[2] = state_msg->body_z[i];

    tf2::Quaternion quaternion;
    quaternion.setRPY(state_msg->body_roll[i], state_msg->body_pitch[i], state_msg->body_yaw[i]);
    ini_state.q[3] = quaternion.x();
    ini_state.q[4] = quaternion.y();
    ini_state.q[5] = quaternion.z();
    ini_state.q[6] = quaternion.w();

    ini_state.q[7 + 0] = state_msg->leg_l_q0[i];
    ini_state.q[7 + 1] = state_msg->leg_l_q1[i];
    ini_state.q[7 + 2] = state_msg->leg_l_q2[i];
    ini_state.q[7 + 3] = state_msg->leg_l_q3[i];
    ini_state.q[7 + 4] = state_msg->leg_l_q4[i];
    ini_state.q[7 + 5] = state_msg->leg_l_q5[i];

    ini_state.q[13 + 0] = state_msg->leg_r_q0[i];
    ini_state.q[13 + 1] = state_msg->leg_r_q1[i];
    ini_state.q[13 + 2] = state_msg->leg_r_q2[i];
    ini_state.q[13 + 3] = state_msg->leg_r_q3[i];
    ini_state.q[13 + 4] = state_msg->leg_r_q4[i];
    ini_state.q[13 + 5] = state_msg->leg_r_q5[i];

    data_marker_humanoid.mpc_predict.push_back(ini_state);
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("marker_visualizer");
  g_Node = node;
  rclcpp::Rate loop(500);
  RCLCPP_INFO(node->get_logger(), "1111111111111111111111111 marker_visualizer started\n");
  // subscribe key ros msg
  auto state_sub = node->create_subscription<lcm_to_ros2::msg::StateEstimatorLcmt>(
    "/robot_lcm/state_estimator", 5, StateCallback);
  auto leg_state_sub = node->create_subscription<lcm_to_ros2::msg::LegControlDataLcmt>(
    "/robot_lcm/leg_control_data", 5, legStateCallback);
  auto waist_state_sub = node->create_subscription<lcm_to_ros2::msg::WaistControlDataLcmt>(
    "/robot_lcm/waist_control_data", 5, waistStateCallback);
  auto head_state_sub = node->create_subscription<lcm_to_ros2::msg::HeadControlDataLcmt>(
    "/robot_lcm/head_control_data", 5, headStateCallback);
  auto arm_state_sub = node->create_subscription<lcm_to_ros2::msg::ArmControlDataLcmt>(
    "/robot_lcm/arm_control_data", 5, armStateCallback);

  auto fsm_sub = node->create_subscription<lcm_to_ros2::msg::LcmResponseLcmt>(
    "/robot_lcm/lcm_response", 5, fsmCallback);
  auto wbc_wrench_sub = node->create_subscription<lcm_to_ros2::msg::WbcTestDataT>(
    "/robot_lcm/wbc_lcm_data", 5, wbcWrenchCallback);
  
  auto trajectory_joint_sub = node->create_subscription<lcm_to_ros2::msg::TrajectoryJointLcmt>(
    "/robot_lcm/trajectory_joint", 5, jointTrajectoryCallback);

  // different marker publisher
  auto rviz_marker_base_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("magic/rviz_markers_base_visualizer", 1);
  magic::MarkerBase marker_base_rviz;

  auto rviz_marker_humanoid_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("magic/rviz_markers_huamnoid_visualizer", 1);
  std::string package_path = ament_index_cpp::get_package_share_directory("magic_vis_ros2");
  std::string urdf_path = package_path + "/p5_humanoid.urdf";
  magic::MarkerHumanoid marker_huamnoid_rviz(urdf_path);
  data_marker_humanoid.cur_state.init(marker_huamnoid_rviz.pino_model->getModel().nv);

  double massF = 0.;
  for (auto e : marker_huamnoid_rviz.pino_model->getModel().inertias) {
    massF += e.mass();
  }
  massF *= -marker_huamnoid_rviz.pino_model->getModel().gravity981[2];
  std::cout << "robot total mass: " << massF << std::endl;

  while(rclcpp::ok()) {
 //   RCLCPP_INFO(node->get_logger(), "vvvvvvvvvvvvvvvvvvvv marker_visualizer started\n");
  //  if (f1 && f2 && f3 && f4 && f5) {
    // basic marker
    marker_base_rviz.data = data_marker_base;
    marker_base_rviz.BuildRobotTF();
    auto rviz_marker_base_msg = marker_base_rviz.BuildRobotState();
    rviz_marker_base_pub->publish(rviz_marker_base_msg);
    
    // humanoid marker
    // data_marker_humanoid.mpc_predict.clear();
    // magic::HumanState ini_state = data_marker_humanoid.cur_state;
    // for (int i=0; i<50; i++) {
    //   ini_state.q[0] = data_marker_humanoid.cur_state.q[0] + 3. * sin(M_PI * i / 100.);
    //   ini_state.q[1] = data_marker_humanoid.cur_state.q[1] + 0.5 * sin(M_PI * i / 10.);
    //   data_marker_humanoid.mpc_predict.push_back(ini_state);
    // }

    // marker_huamnoid_rviz.data = data_marker_humanoid;
    // marker_huamnoid_rviz.DataHandle(fsm==9 || fsm==88);
    // marker_huamnoid_rviz.BuildRobotTF();
    // auto rviz_marker_humanoid_msg = marker_huamnoid_rviz.BuildRobotState();
    // rviz_marker_humanoid_pub->publish(rviz_marker_humanoid_msg);
  //  }
    
    loop.sleep();
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}