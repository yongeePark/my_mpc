#include <my_mpc/mympc.h>


namespace mympc {
  // constructor
  ModelPredictiveController::ModelPredictiveController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_param) 
  :nh_(nh), nh_param_(nh_param)
  {
    std::cout<<"[mympc.cc] iniitalizaing mpc controller"<<std::endl;
    acado_initializeSolver();
    W_.setZero();
    WN_.setZero();
    state_.setZero();
    input_.setZero();
    reference_.setZero();
    referenceN_.setZero();
    
    // initial goal is O
    position_ref_ << 0.0, 0.0, 1.0;
    velocity_ref_ << 0.0, 0.0, 0.0;

    initialized_parameters_ = setControllerParameters();
    
    std::cout<<"[mympc.cc] initialization is done"<<std::endl;
  }

  bool ModelPredictiveController::setControllerParameters()
  {
    std::cout<<"reading parameter"<<std::endl;

    std::string nodename = "/controller_node";
    std::vector<double> drag_coefficients;
    std::vector<double> r_command;
    double q_position_x,q_position_y,q_position_z;
    double q_velocity_x,q_velocity_y,q_velocity_z;
    double q_attitude_pos, q_attitude_vel;
    if (!nh_param_.getParam(nodename + "/mass",mass_))
    {
        std::cout<<"[Error] Please set [mass] parameter"<<std::endl;
        return false;
    }

    if (!nh_param_.getParam(nodename + "/roll_time_constant",roll_time_constant_))
    {
        std::cout<<"[Error] Please set [roll_time_constant] parameter"<<std::endl;
        return false;
    }
    if (!nh_param_.getParam(nodename + "/pitch_time_constant",pitch_time_constant_))
    {
        std::cout<<"[Error] Please set [pitch_time_constant] parameter"<<std::endl;
        return false;
    }
    if (!nh_param_.getParam(nodename + "/roll_gain",roll_gain_))
    {
        std::cout<<"[Error] Please set [roll_gain] parameter"<<std::endl;
        return false;
    }
    if (!nh_param_.getParam(nodename + "/pitch_gain",pitch_gain_))
    {
        std::cout<<"[Error] Please set [pitch_gain] parameter"<<std::endl;
        return false;
    }

    if (!nh_param_.getParam(nodename + "/drag_coefficients",drag_coefficients))
    {
        std::cout<<"[Error] Please set [drag_coefficients] parameter"<<std::endl;
        return false;
    }
    drag_coefficients_ << drag_coefficients.at(0),drag_coefficients.at(1),drag_coefficients.at(2);

    if (!nh_param_.getParam(nodename + "/q_position_x",q_position_x))
    {
        std::cout<<"[Error] Please set [q_position_x] parameter"<<std::endl;
        return false;
    }
    if (!nh_param_.getParam(nodename + "/q_position_y",q_position_y))
    {
        std::cout<<"[Error] Please set [q_position_Y] parameter"<<std::endl;
        return false;
    }
    if (!nh_param_.getParam(nodename + "/q_position_z",q_position_z))
    {
        std::cout<<"[Error] Please set [q_position_z] parameter"<<std::endl;
        return false;
    }
    q_position_ << q_position_x,q_position_y,q_position_z;

    if (!nh_param_.getParam(nodename + "/q_velocity_x",q_velocity_x))
    {
        std::cout<<"[Error] Please set [q_velocity_x] parameter"<<std::endl;
        return false;
    }
    if (!nh_param_.getParam(nodename + "/q_velocity_y",q_velocity_y))
    {
        std::cout<<"[Error] Please set [q_velocity_Y] parameter"<<std::endl;
        return false;
    }
    if (!nh_param_.getParam(nodename + "/q_velocity_z",q_velocity_z))
    {
        std::cout<<"[Error] Please set [q_velocity_z] parameter"<<std::endl;
        return false;
    }
    q_velocity_ << q_velocity_x,q_velocity_y,q_velocity_z;

    if (!nh_param_.getParam(nodename + "/q_attitude_pos",q_attitude_pos))
    {
        std::cout<<"[Error] Please set [q_attitude_pos] parameter"<<std::endl;
        return false;
    }
    if (!nh_param_.getParam(nodename + "/q_attitude_vel",q_attitude_vel))
    {
        std::cout<<"[Error] Please set [q_attitude_vel] parameter"<<std::endl;
        return false;
    }
    q_attitude_ << q_attitude_pos,q_attitude_vel;
    
    if (!nh_param_.getParam(nodename + "/r_command",r_command))
    {
        std::cout<<"[Error] Please set [r_command] parameter"<<std::endl;
        return false;
    }
    r_command_ << r_command.at(0), r_command.at(1),r_command.at(2);

    if (!nh_param_.getParam(nodename + "/roll_limit",roll_limit_))
    {
        std::cout<<"[Error] Please set [roll_limit] parameter"<<std::endl;
        return false;
    }
    if (!nh_param_.getParam(nodename + "/pitch_limit",pitch_limit_))
    {
        std::cout<<"[Error] Please set [pitch_limit] parameter"<<std::endl;
        return false;
    }
    if (!nh_param_.getParam(nodename + "/yaw_rate_limit",yaw_rate_limit_))
    {
        std::cout<<"[Error] Please set [yaw_rate_limit] parameter"<<std::endl;
        return false;
    }
    yaw_rate_limit_ = yaw_rate_limit_ / 180 * M_PI;

    if (!nh_param_.getParam(nodename + "/thrust_min",thrust_min_))
    {
        std::cout<<"[Error] Please set [thrust_min] parameter"<<std::endl;
        return false;
    }
    if (!nh_param_.getParam(nodename + "/thrust_max",thrust_max_))
    {
        std::cout<<"[Error] Please set [thrust_max] parameter"<<std::endl;
        return false;
    }

    if (!nh_param_.getParam(nodename + "/k_yaw",k_yaw_))
    {
        std::cout<<"[Error] Please set [k_yaw] parameter"<<std::endl;
        return false;
    }

    for (int i = 0; i < ACADO_N + 1; i++) {
    acado_online_data_.block(i, 0, 1, ACADO_NOD) << roll_time_constant_, roll_gain_, pitch_time_constant_, pitch_gain_, drag_coefficients_(
        0), drag_coefficients_(1);
    }

    Eigen::Map<Eigen::Matrix<double, ACADO_NOD, ACADO_N + 1>>(const_cast<double*>(acadoVariables.od)) =
      acado_online_data_.transpose();
    
    // set W matrix
    W_.block(0, 0, 3, 3) = q_position_.asDiagonal();
    W_.block(3, 3, 3, 3) = q_velocity_.asDiagonal();
    W_.block(6, 6, 2, 2) = q_attitude_.asDiagonal();
    W_.block(8, 8, 3, 3) = r_command_.asDiagonal();

    // set WN matrix
    WN_ = solveCARE((Eigen::VectorXd(6) << q_position_, q_velocity_).finished().asDiagonal(),
                  r_command_.asDiagonal());
    Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_NY>>(const_cast<double*>(acadoVariables.W)) =
      W_.transpose();
    Eigen::Map<Eigen::Matrix<double, ACADO_NYN, ACADO_NYN>>(const_cast<double*>(acadoVariables.WN)) =
      WN_.transpose();


    for (size_t i = 0; i < ACADO_N; ++i) {
      acadoVariables.lbValues[3 * i] = -roll_limit_ / 180 * M_PI;       // min roll
      acadoVariables.lbValues[3 * i + 1] = -pitch_limit_ / 180 * M_PI;  // min pitch
      acadoVariables.lbValues[3 * i + 2] = thrust_min_;    // min thrust
      acadoVariables.ubValues[3 * i] = roll_limit_ / 180 * M_PI;        // max roll
      acadoVariables.ubValues[3 * i + 1] = pitch_limit_ / 180 * M_PI;   // max pitch
      acadoVariables.ubValues[3 * i + 2] = thrust_max_;    // max thrust
    }   
    return true;
  }

  void ModelPredictiveController::setGoal(const geometry_msgs::PoseStamped& pose)
  {
    position_ref_(0) = pose.pose.position.x;
    position_ref_(1) = pose.pose.position.y;
    position_ref_(2) = pose.pose.position.z;

    // rpy[0] = std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
    //                         1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));

    // rpy[1] = std::asin(2.0 * (q.w() * q.y() - q.z() * q.x()));

    // rpy[2] = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
    //             1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    yaw_ref_ = std::atan2(2.0 * ( pose.pose.orientation.w * pose.pose.orientation.z + pose.pose.orientation.x * pose.pose.orientation.y),
    1.0 - 2.0 * (pose.pose.orientation.y * pose.pose.orientation.y + pose.pose.orientation.z * pose.pose.orientation.z) );
  }
  // Set current Odometry
  void ModelPredictiveController::setOdometry(const nav_msgs::Odometry& odometry)
  {
    static nav_msgs::Odometry previous_odometry = odometry;
    odometry_ = odometry;
    current_yaw_ = std::atan2(2.0 * ( odometry.pose.pose.orientation.w * odometry.pose.pose.orientation.z + odometry.pose.pose.orientation.x * odometry.pose.pose.orientation.y),
    1.0 - 2.0 * (odometry.pose.pose.orientation.y * odometry.pose.pose.orientation.y + odometry.pose.pose.orientation.z * odometry.pose.pose.orientation.z) );
    if (!received_first_odometry_) {
      Eigen::VectorXd x0(ACADO_NX);
      received_first_odometry_ = true;
      Eigen::Vector3d euler_angles;
      Eigen::Quaterniond q;
      q.x() = odometry.pose.pose.orientation.x;
      q.y() = odometry.pose.pose.orientation.y;
      q.z() = odometry.pose.pose.orientation.z;
      q.w() = odometry.pose.pose.orientation.w;
    
      euler_angles = q.toRotationMatrix().eulerAngles(0,1,2);

      // transfrom based on yaw angle
      double current_roll_rot, current_pitch_rot;
      current_roll_rot   =  cos(euler_angles(2))*euler_angles(0) + sin(euler_angles(2)) * euler_angles(1);
      current_pitch_rot  = -sin(euler_angles(2))*euler_angles(0) + cos(euler_angles(2)) * euler_angles(1);


      x0 << odometry.twist.twist.linear.x,
        odometry.twist.twist.linear.y,
        odometry.twist.twist.linear.z,
        current_roll_rot,
        current_pitch_rot,
        0,
        odometry.pose.pose.position.x,
        odometry.pose.pose.position.y,
        odometry.pose.pose.position.z;
  
      initializeAcadoSolver(x0);
    }
  }

  void ModelPredictiveController::initializeAcadoSolver(Eigen::VectorXd x0)
  {
    for (int i = 0; i < ACADO_N + 1; i++)
    { state_.block(i, 0, 1, ACADO_NX) << x0.transpose();  }

    Eigen::Map<Eigen::Matrix<double, ACADO_NX, ACADO_N + 1>>(const_cast<double*>(acadoVariables.x)) =
        state_.transpose();
    Eigen::Map<Eigen::Matrix<double, ACADO_NU, ACADO_N>>(const_cast<double*>(acadoVariables.u)) =
        input_.transpose();
    Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_N>>(const_cast<double*>(acadoVariables.y)) =
        reference_.transpose();
    Eigen::Map<Eigen::Matrix<double, ACADO_NYN, 1>>(const_cast<double*>(acadoVariables.yN)) =
        referenceN_.transpose();
  }

// =============================================================================================================================================
 //find control input by solving optimization problem.
  void ModelPredictiveController::calculateRollPitchYawrateThrustCommand(Eigen::Vector4d* ref_attitude_thrust)
  {
    assert(ref_attitude_thrust != nullptr);
    assert(initialized_parameters_ == true);
  
  Eigen::Matrix<double, ACADO_NX, 1> x_0;
  Eigen::Vector3d current_rpy;
  for (size_t i = 0; i < ACADO_N; i++) {

    reference_.block(i, 0, 1, ACADO_NY) << position_ref_[0], position_ref_[1], position_ref_[2],
                                           velocity_ref_[0], velocity_ref_[1], velocity_ref_[2],
                                           0, 0, 0, 0, 0;
  }

  referenceN_ << position_ref_[0], position_ref_[1], position_ref_[2],
                 velocity_ref_[0], velocity_ref_[1], velocity_ref_[2];
  //acado_online_data_.block(ACADO_N, ACADO_NOD - 3, 1, 3) << estimated_disturbances.transpose();

  Eigen::Vector3d euler_angles;
  Eigen::Quaterniond q;
  q.x() = odometry_.pose.pose.orientation.x;
  q.y() = odometry_.pose.pose.orientation.y;
  q.z() = odometry_.pose.pose.orientation.z;
  q.w() = odometry_.pose.pose.orientation.w;

  current_yaw_ = std::atan2(2.0 * ( q.w() * q.z() + q.x() * q.y()),
    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()) );
  // euler_angles = q.toRotationMatrix().eulerAngles(0,1,2);

  euler_angles << std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                          1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y())),
      std::asin(2.0 * (q.w() * q.y() - q.z() * q.x())),
      std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
            1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

  // Eigen::Vector3d euler_angles;
  // Eigen::Quaterniond q;
  // q.x() = odometry.pose.pose.orientation.x;
  // q.y() = odometry.pose.pose.orientation.y;
  // q.z() = odometry.pose.pose.orientation.z;
  // q.w() = odometry.pose.pose.orientation.w;

  // euler_angles = q.toRotationMatrix().eulerAngles(0,1,2);


  // std::cout<<"current position"<<std::endl;
  // std::cout<<odometry_.pose.pose.position.x<<", "<<odometry_.pose.pose.position.y<<", "<<odometry_.pose.pose.position.z<<std::endl;
  // std::cout<<"euler_angles"<<std::endl;
  // std::cout<<euler_angles(0)<<", "<<euler_angles(1)<<", "<<euler_angles(2) <<std::endl;

  adjust_angle(euler_angles(0),M_PI);
  adjust_angle(euler_angles(1),M_PI);
  adjust_angle(euler_angles(2),M_PI*2);

  //velocity transform!
  Eigen::Vector3d current_velocity, world_velocity;
  current_velocity << odometry_.twist.twist.linear.x, odometry_.twist.twist.linear.y, odometry_.twist.twist.linear.z;
  world_velocity = q * current_velocity;


  x_0 <<world_velocity(0),
        world_velocity(1),
        world_velocity(2),
        euler_angles(0),
        euler_angles(1),
        euler_angles(2),
        // current_roll_rot,
        // current_pitch_rot,
        // 0,
        odometry_.pose.pose.position.x,
        odometry_.pose.pose.position.y,
        odometry_.pose.pose.position.z;

  //set value
  Eigen::Map<Eigen::Matrix<double, ACADO_NX, 1>>(const_cast<double*>(acadoVariables.x0)) = x_0;
  Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_N>>(const_cast<double*>(acadoVariables.y)) =
      reference_.transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NYN, 1>>(const_cast<double*>(acadoVariables.yN)) =
      referenceN_.transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NOD, ACADO_N + 1>>(const_cast<double*>(acadoVariables.od)) =
      acado_online_data_.transpose();

  // Solve
  
  // ros::WallTime time_before_solving = ros::WallTime::now();
  acado_preparationStep();

  int acado_status = acado_feedbackStep();
  // solve_time_average_ += (ros::WallTime::now() - time_before_solving).toSec() * 1000.0;

  double roll_ref = acadoVariables.u[0];
  double pitch_ref = acadoVariables.u[1];
  double thrust_ref = acadoVariables.u[2];

  // std::cout<<"========================================"<<std::endl;
  // std::cout<<"roll ref : "<<roll_ref<<std::endl<<
  //            "pitch ref : "<<pitch_ref<<std::endl;

  // std::cout<<"optimization result"<<std::endl;
  // std::cout<<"Roll   : "<<roll_ref<<
  //            "Pitch  : "<<pitch_ref<<
  //            "Thrust : "<<thrust_ref<<std::endl;
  

  if (std::isnan(roll_ref) || std::isnan(pitch_ref) || std::isnan(thrust_ref)
      || acado_status != 0) {
    ROS_WARN_STREAM("Nonlinear MPC: Solver failed with status: " << acado_status);
    ROS_WARN("reinitializing...");
    initializeAcadoSolver (x_0);
    *ref_attitude_thrust << 0, 0, 0, kGravity * mass_;
    return;
  }

  // command_roll_pitch_yaw_thrust_ << roll_ref, pitch_ref, yaw_ref_.front(), thrust_ref;

  state_ = Eigen::Map<Eigen::Matrix<double, ACADO_N + 1, ACADO_NX, Eigen::RowMajor>>(
      acadoVariables.x);

  // yaw controller
  double yaw_error = yaw_ref_ - current_yaw_;
  if (abs(yaw_error) > M_PI )
  {
    if (yaw_error > 0){yaw_error = - 2 * M_PI + yaw_error;}
    else {yaw_error = 2 * M_PI + yaw_error;}
  }
  double yaw_diff = k_yaw_ * yaw_error;

  if (abs(yaw_diff) > yaw_rate_limit_)
  {
      if(yaw_diff > 0 ){yaw_diff = yaw_rate_limit_;}
      else {yaw_diff = -yaw_rate_limit_;}
  }
  double yaw_cmd = yaw_diff + current_yaw_;  // feed-forward yaw_rate cmd

  // transfrom based on yaw angle
  double roll_ref_rot, pitch_ref_rot;
  roll_ref_rot = cos(yaw_cmd)*roll_ref + sin(yaw_cmd) * pitch_ref;
  pitch_ref_rot = -sin(yaw_cmd)*roll_ref + cos(yaw_cmd) * pitch_ref;

  // *ref_attitude_thrust = Eigen::Vector4d(roll_ref_rot, pitch_ref_rot, yaw_cmd, mass_ * thrust_ref);
  *ref_attitude_thrust = Eigen::Vector4d(roll_ref, pitch_ref, yaw_cmd, mass_ * thrust_ref);


  static int counter = 0;
  if (counter >= 100) {
    // ROS_INFO_STREAM("average solve time: " << solve_time_average_ / counter << " ms");
    // solve_time_average_ = 0.0;

    // ROS_INFO_STREAM("Controller loop time : " << diff_time*1000.0 << " ms");

    // ROS_INFO_STREAM(
        // "roll ref: " << command_roll_pitch_yaw_thrust_(0) << "\t" << "pitch ref : \t" << command_roll_pitch_yaw_thrust_(1) << "\t" << "yaw ref : \t" << command_roll_pitch_yaw_thrust_(2) << "\t" << "thrust ref : \t" << command_roll_pitch_yaw_thrust_(3) << "\t" << "yawrate ref : \t" << yaw_rate_cmd);
    std::cout<<"============================================"<<std::endl;
    std::cout<<"[mympc.cc] reference info"<<std::endl;
    std::cout<<"acado solver status : " << acado_status<<std::endl;
    // std::cout<<
    
    // "yaw ref : \t"       << (*ref_attitude_thrust)[2] << std::endl<<
    // "thrust ref : \t"    << (*ref_attitude_thrust)[3] << std::endl<<
    
    std::cout<<"============================================"<<std::endl;
    std::cout<<
    "Current Roll : "    << euler_angles(0) * 180 / M_PI << std::endl <<
    "Current Pitch : "    << euler_angles(1) * 180 / M_PI << std::endl <<
    // "current_roll_rot : "<<current_roll_rot * 180 / M_PI<<std::endl<<
    // "current_pitch_rot : "<<current_pitch_rot * 180 / M_PI<<std::endl<<
    "============================================"<<std::endl<<
    "roll ref : \t"      << roll_ref * 180 / M_PI << std::endl<<
    "pitch ref : \t"     << pitch_ref * 180 / M_PI << std::endl<<
    "roll_ref_rot : "<<roll_ref_rot * 180 / M_PI<<std::endl<<
    "pitch_ref_rot : "<<pitch_ref_rot * 180 / M_PI<<std::endl<<
    "============================================"<<std::endl<<
    "yaw_ref : "<<yaw_ref_ * 180 / M_PI<<std::endl<<
    "current yaw : "<<current_yaw_ * 180 / M_PI<<std::endl<<
    "yaw_error : "<<yaw_diff * 180 / M_PI<<std::endl<<
    "yaw_cmd : "<<yaw_cmd * 180 / M_PI<<std::endl;
    std::cout<<"Current pose : ["<<odometry_.pose.pose.position.x<<", "<<odometry_.pose.pose.position.y<<", "<<odometry_.pose.pose.position.z<<"]"<<std::endl;
    std::cout<<"Current goal : ["<<position_ref_[0]<<", "<<position_ref_[1]<<", "<<position_ref_[2]<<"]"<<std::endl;
    
    counter = 0;
  }
  counter++;
  }

void ModelPredictiveController::getPredictedState(visualization_msgs::Marker& marker)
{
  for (uint32_t i = 0; i < ACADO_N+1; ++i)
  {
    geometry_msgs::Point p;
    p.x = state_(i,6);
    p.y = state_(i,7);
    p.z = state_(i,8);

    marker.points.push_back(p);

  }
 // pub_marker_.publish(marker);
}


  Eigen::MatrixXd ModelPredictiveController::solveCARE(Eigen::MatrixXd Q, Eigen::MatrixXd R)
  {
    // Define system matrices
    Eigen::MatrixXd A;
    A.resize(6, 6);
    A.setZero();

    A.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();
    A.block(3, 3, 3, 3) = -1.0 * drag_coefficients_.asDiagonal();

    Eigen::MatrixXd B;
    B.resize(6, 3);
    B.setZero();

    B(3, 1) = kGravity;
    B(4, 0) = -1.0 * kGravity;
    B(5, 2) = 1.0;

    Eigen::MatrixXd G = B * R.inverse() * B.transpose();

    Eigen::MatrixXd z11 = A;
    Eigen::MatrixXd z12 = -1.0 * G;
    Eigen::MatrixXd z21 = -1.0 * Q;
    Eigen::MatrixXd z22 = -1.0 * A.transpose();

    Eigen::MatrixXd Z;
    Z.resize(z11.rows() + z21.rows(), z11.cols() + z12.cols());
    Z << z11, z12, z21, z22;

    int n = A.cols();
    Eigen::MatrixXd U(2 * n, 2 * n);  // Orthogonal matrix from Schur decomposition
    Eigen::VectorXd WR(2 * n);
    Eigen::VectorXd WI(2 * n);
    lapack_int sdim = 0;  // Number of eigenvalues for which sort is true
    lapack_int info;
    info = LAPACKE_dgees(LAPACK_COL_MAJOR,  // Eigen default storage order
        'V',               // Schur vectors are computed
        'S',               // Eigenvalues are sorted
        select_lhp,        // Ordering callback
        Z.rows(),          // Dimension of test matrix
        Z.data(),          // Pointer to first element
        Z.rows(),          // Leading dimension (column stride)
        &sdim,             // Number of eigenvalues sort is true
        WR.data(),         // Real portion of eigenvalues
        WI.data(),         // Complex portion of eigenvalues
        U.data(),          // Orthogonal transformation matrix
        Z.rows());         // Dimension of Z

    Eigen::MatrixXd U11 = U.block(0, 0, n, n).transpose();
    Eigen::MatrixXd U21 = U.block(n, 0, n, n).transpose();

    return U11.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(U21).transpose();

    // return A;
  }



}
