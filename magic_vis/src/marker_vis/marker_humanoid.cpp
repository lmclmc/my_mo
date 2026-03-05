#include <magic_vis/marker_vis/marker_humanoid.h>

namespace magic {

MarkerHumanoid::MarkerHumanoid(std::string urdf)
{
  pino_model = new ModelHumanoid(urdf);

  G.resize(0., 12, 12);
  g0.resize(0., 12);

  CE.resize(0., 12, 6);
  ce0.resize(0., 6);

  CI.resize(0., 12, 20);
  ci0.resize(0., 20);

  for (int i=0; i<4; i++) {
    G[3*i][3*i] = 1.;
    G[3*i+1][3*i+1] = 1.;
    G[3*i+2][3*i+2] = ((i==0 || i==1)? 0.1 : 0.2);

    CE[3*i][0] = 1.;
    CE[3*i+1][1] = 1.;
    CE[3*i+2][2] = 1.;
  }

  // print G
  // std::printf("G\n");
  // for (int r=0; r<12; r++) {
  //   for (int c=0; c<12; c++) {
  //     std::printf("%5f\t", G[c][r]);
  //   }
  //   std::printf("%5f\n", g0[r]);
  // }
}

MarkerHumanoid::~MarkerHumanoid()
{
  delete pino_model;
}

void 
MarkerHumanoid::BuildRobotTF() const
{

}

MarkerHumanoid::MarkerArray
MarkerHumanoid::BuildRobotState() const
{
  MarkerArray msg;

  MarkerVec support = CreateSupport(data.support_center, data.support_vert);
  msg.markers.insert(msg.markers.begin(), support.begin(), support.end());

  // contact footstep and its GRF
  for (int i=0; i<2; i++) {
    MarkerVec fse = CreateFootstepElement(data.cur_state.foot_corner.middleCols(4*i, 4), 
                                          data.cur_state.contact[i]>1e-3, data.cur_state.wrench_decomp.middleCols(4*i, 4).norm()>1.);
    msg.markers.insert(msg.markers.begin(), fse.begin(), fse.end());

    MarkerVec desf = CreateFootstepForce(data.cur_state.foot_corner.middleCols(4*i, 4), 
                                          data.cur_state.wrench_des_decomp.middleCols(4*i, 4), "des", data.cur_state.contact[i]>1e-3);
    msg.markers.insert(msg.markers.begin(), desf.begin(), desf.end());

    MarkerVec curf = CreateFootstepForce(data.cur_state.foot_corner.middleCols(4*i, 4), 
                                          data.cur_state.wrench_decomp.middleCols(4*i, 4), "act", data.cur_state.contact[i]>1e-3);
    msg.markers.insert(msg.markers.begin(), curf.begin(), curf.end());
  }

  // mpc base and ee_foot trajectories
  MarkerVec mpct = CreateMpcTrajectories();
  msg.markers.insert(msg.markers.begin(), mpct.begin(), mpct.end());

  int id = 0;
  for (Marker& m : msg.markers) {
    m.header.frame_id = frame_id_;
    m.id = id++;
  }

  return msg;
}

void
MarkerHumanoid::DataHandle(bool wbc_fsm)
{
  // current state
  SingleStateHandle(data.cur_state, std::vector<bool>{true, wbc_fsm, wbc_fsm});

  // mpc trajectory
  base.clear();
  footl.clear();
  footr.clear();
  for (auto e : data.mpc_predict) {
    SingleStateHandle(e);
    base.push_back(e.q.head(3));
    footl.push_back(e.foot.col(0));
    footr.push_back(e.foot.col(1));
  }
}

void
MarkerHumanoid::SingleStateHandle(HumanState& state, std::vector<bool> cal_flag)
{
  pino_model->updateKinematics(state.q, state.dq);

  if (cal_flag[0]) {
    for (int i=0; i<8; i++) 
      state.foot_corner.col(i) = pino_model->framePosition(foot_corner_name[i]).translation();

    for (int i=0; i<2; i++)
      state.foot.col(i) = pino_model->framePosition(foot_name[i]).translation();
  }
  
  for (int i=0; i<2; i++) {
    if (state.contact[i]>1e-3) {
      if (cal_flag[1])
        state.wrench_des_decomp.middleCols(4*i, 4) = 
              WrenchDecompose(state.wrench_des.col(i), state.foot.col(i), state.foot_corner.middleCols(4*i, 4));

      if (cal_flag[2])
        state.wrench_decomp.middleCols(4*i, 4) = 
              WrenchDecompose(state.wrench.col(i), state.foot.col(i), state.foot_corner.middleCols(4*i, 4));
    }
  }
}

MarkerHumanoid::Mat34 
MarkerHumanoid::WrenchDecompose(Wrench wrench, Vector3d foot, Mat34 corner)
{
  Mat34 res = Mat34::Zero();

  if (wrench.norm()<1)
    return res;

  for (int i=0; i<4; i++) {
    Eigen::Matrix3d r_skew = skew(corner.col(i) - foot);
    for (int c=0; c<3; c++) {
      for (int r=0; r<3; r++) {
        CE[3*i+r][3+c] = r_skew(c, r);
      }
    }
  }

  for (int i=0; i<6; i++) {
    ce0[i] = -wrench[i];
  }

  // print CE
  // std::printf("CE\n");
  // for (int r=0; r<6; r++) {
  //   for (int c=0; c<12; c++) {
  //     std::printf("%5f\t", CE[c][r]);
  //   }
  //   std::printf("%5f\n", ce0[r]);
  // }

  for (int i=0; i<4; i++) {
    CI[3*i+0][5*i+0] =  1.;  CI[3*i+2][5*i+0] = CONTACT_MU;
    CI[3*i+0][5*i+1] = -1.;  CI[3*i+2][5*i+1] = CONTACT_MU;
    CI[3*i+1][5*i+2] =  1.;  CI[3*i+2][5*i+2] = CONTACT_MU;
    CI[3*i+1][5*i+3] = -1.;  CI[3*i+2][5*i+3] = CONTACT_MU;
    CI[3*i+2][5*i+4] =  1.;
  }

  // print CI
  // std::printf("CI\n");
  // for (int r=0; r<20; r++) {
  //   for (int c=0; c<12; c++) {
  //     std::printf("%5f\t", CI[c][r]);
  //   }
  //   std::printf("%5f\n", ci0[r]);
  // }

  double f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z);
  if (f == std::numeric_limits<double>::infinity()) {
    std::printf("wrench to redundant forces: QP is infeasible, using average Fz!\n");
    std::cout << "wrench: " << wrench.transpose() << std::endl;
    res.row(2) << 0.25*wrench.row(2), 0.25*wrench.row(2), 0.25*wrench.row(2), 0.25*wrench.row(2);
  }
  else {
    for (int r=0; r<3; r++) {
      for (int c=0; c<4; c++) {
        res(r, c) = z[r + 3*c];
      }
    }

    // for (int i=0; i<12; i++) std::cout << z[i] << " ";
    // std::cout << "\nres:\n" << res << std::endl;
  }

  // for (int i=0; i<4; i++) {
  //   Eigen::Matrix3d r_skew = skew(corner.col(i) - foot);
  //   std::cout << i << "\n" << r_skew << "\n" << r_skew * res.col(i) << std::endl;
  // }

  return res;
}

MarkerHumanoid::MarkerVec 
MarkerHumanoid::CreateSupport(const Vector3d& pos, const EEPos& pos_W) const
{
  MarkerVec vec;

  Marker support = CreatePolygonArea(pos_W);
  support.ns = "support_polygon";
  support.color = color.black;
  support.color.a = (pos_W.size()==0? 0 : 0.8);
  vec.push_back(support);

  Marker msup = CreateSupportMarginPoint(pos);
  msup.color.a = (pos_W.size()==0? 0 : 0.8);
  vec.push_back(msup);

  return vec;
}

MarkerHumanoid::MarkerVec 
MarkerHumanoid::CreateFootstepElement(const Mat34 corner, const bool vis, const bool vis_cone) const
{
  // data format convert
  EEPos ee_pos;
  ee_pos.clear();

  for (int i=0; i<4; i++) {
    ee_pos.push_back(corner.col(i));
  }

  // friction cone
  bool fric_vis = (vis && vis_cone); 
  MarkerVec vec = CreateFrictionCones(ee_pos, ContactState{fric_vis, fric_vis, fric_vis, fric_vis}, CONTACT_MU);

  // footstep
  Marker footstep = CreatePolygonArea(ee_pos);
  footstep.ns = "footstep_polygon";
  footstep.color = color.green;
  footstep.color.a = (vis? 0.5 : 0.);
  vec.push_back(footstep);

  return vec;
}

MarkerHumanoid::MarkerVec
MarkerHumanoid::CreateFootstepForce(const Mat34 corner, const Mat34 force, const std::string name, const bool vis) const
{
  // data format convert
  EEPos ee_pos;
  EEForces ee_force;
  ee_pos.clear();
  ee_force.clear();

  for (int i=0; i<4; i++) {
    ee_pos.push_back(corner.col(i));
    ee_force.push_back(force.col(i));
  }

  // foot forces
  MarkerVec vec = CreateEEForces(ee_force, ee_pos, "footstep_force_" + name, vis);
  if (name == "act") {
    for (auto &e : vec) {
      e.color.r = color.blue.r;
      e.color.g = color.blue.g;
      e.color.b = color.blue.b;
    }
  }
  else if (name == "mpc") {
    for (auto &e : vec) {
      e.color.r = color.green.r;
      e.color.g = color.green.g;
      e.color.b = color.green.b;
    }
  }

  return vec;
}

MarkerHumanoid::MarkerVec 
MarkerHumanoid::CreateMpcTrajectories() const
{
  MarkerVec vec;

  Marker base_traj = CreateTrajectory(base);
  base_traj.ns = "mpc_base";
  base_traj.color = color.red;
  base_traj.color.a = (base.size()>0? 0.7 : 0.);
  vec.push_back(base_traj);

  Marker footl_traj = CreateTrajectory(footl);
  footl_traj.ns = "mpc_footl";
  footl_traj.color = color.brown;
  footl_traj.color.a = (footl.size()>0? 0.7 : 0.);
  vec.push_back(footl_traj);

  Marker footr_traj = CreateTrajectory(footr);
  footr_traj.ns = "mpc_footr";
  footr_traj.color = color.brown;
  footr_traj.color.a = (footr.size()>0? 0.7 : 0.);

  vec.push_back(footr_traj);

  return vec;
}

} /* namespace magic */