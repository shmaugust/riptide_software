#include "riptide_controllers/thruster_controller.h"

#undef debug
#undef report
#undef progress

#define PI 3.141592653
#define GRAVITY 9.81 //[m/s^2]
#define WATER_DENSITY 1000.0 //[kg/m^3]

// **Please keep these in case they get deleted from the vehicle_properties.yaml file
/*#define Ixx 0.52607145
#define Iyy 1.50451601
#define Izz 1.62450600*/
double Ixx, Iyy, Izz;

struct vector {
  double x;
  double y;
  double z;
};

// Thrust limits (N):
// These limits cannot be set too low b/c otherwise it will interfere with
// the EOMs and result in additional thrusters turning on to maintain those
// relationships. Ex. surge and sway will kick in and move the vehicle at a diagonal
// when the heave thrust is capped at too low of a number. If these limits are
// laxed, then the solver will not turn on those additional thrusters and the
// output will be as expected.
// NOTE: For the time being, the upper/lower bounds have been REMOVED from the solver
double MIN_THRUST = -20.0;
double MAX_THRUST = 20.0;

// Vehicle mass (kg):
// Updated 5-15-18
double mass;
double weight = mass*GRAVITY;

// Vehcile volume (m^3)
// TODO: Get this value from model
// Updated on 5/11/18
double volume;
double buoyancy = volume * WATER_DENSITY * GRAVITY;

/*// Moments of inertia (kg*m^2)
double Ixx = 0.52607145;
double Iyy = 1.50451601;
double Izz = 1.62450600;*/

// Acceleration commands (m/s^):
double cmdSurge = 0.0;
double cmdSway = 0.0;
double cmdHeave = 0.0;
double cmdRoll = 0.0;
double cmdPitch = 0.0;
double cmdYaw = 0.0;

// Solved Thruster Forces
double surge_port_lo, surge_stbd_lo;
double sway_fwd, sway_aft;
double heave_port_aft, heave_stbd_aft, heave_stbd_fwd, heave_port_fwd;

// Buoyancy Variables
bool isBuoyant, enableHeaveFwd, enableHeaveAft;
double pos_buoyancy_x, pos_buoyancy_y, pos_buoyancy_z;
double buoyancy_depth_thresh, buoyancy_pitch_thresh;

// Rotation Matrices: world to body, and body to world
// Angular Velocity
tf::Matrix3x3 R_w2b, R_b2w;
tf::Vector3 euler_deg, euler_rpy, ang_v;

// Debug variables
geometry_msgs::Vector3Stamped buoyancy_pos;

/*void GetTransform(vector *v, tf::StampedTransform *tform)
{
  v->x = tform->getOrigin().x();
  v->y = tform->getOrigin().y();
  v->z = tform->getOrigin().z();

  return;
}*/

/*** Thruster Positions ***/
// Positions are in meters relative to the center of mass (can be neg. or pos.)
vector pos_surge_port_lo;
vector pos_surge_stbd_lo;
vector pos_sway_fwd;
vector pos_sway_aft;
vector pos_heave_port_fwd;
vector pos_heave_port_aft;
vector pos_heave_stbd_fwd;
vector pos_heave_stbd_aft;

// Buoyancy Location
vector pos_buoyancy;

/*** EQUATIONS ***/////////////////////////////////////////////////////////////
// These equations solve for linear/angular acceleration in all axes

// Linear Equations
struct surge
{
  template <typename T>
  bool operator()(const T *const surge_port_lo, const T *const surge_stbd_lo, T *residual) const
  {
    residual[0] = (surge_port_lo[0] + surge_stbd_lo[0] +
                  (T(R_w2b.getRow(0).z()) * (T(buoyancy) - T(weight))*T(isBuoyant))) / T(mass) -
                  T(cmdSurge);
    return true;
  }
};

struct sway
{
  template <typename T>
  bool operator()(const T *const sway_fwd, const T *const sway_aft, T *residual) const
  {
    residual[0] = (sway_fwd[0] + sway_aft[0] +
                  (T(R_w2b.getRow(1).z()) * (T(buoyancy) - T(weight))*T(isBuoyant))) / T(mass) -
                  T(cmdSway);
    return true;
  }
};

struct heave
{
  template <typename T>
  bool operator()(const T *const heave_port_fwd, const T *const heave_stbd_fwd,
                  const T *const heave_port_aft, const T *const heave_stbd_aft, T *residual) const
  {

      residual[0] = (heave_port_fwd[0] + heave_stbd_fwd[0] + heave_port_aft[0] + heave_stbd_aft[0] +
                    (T(R_w2b.getRow(2).z()) * (T(buoyancy) - T(weight))*T(isBuoyant))) / T(mass) -
                    T(cmdHeave);
    return true;
  }
};

// Angular equations

// Roll
// Thrusters contributing to a POSITIVE moment: sway_fwd, sway_aft, heave_port_fwd, heave_port_aft
// Thrusters contributting to a NEGATIVE moment: heave_stbd_fwd, heave_stbd_aft
// Buoyancy Y and Z components produce moments about x-axis
struct roll
{
  template <typename T>
  bool operator()(const T *const sway_fwd, const T *const sway_aft,
                  const T *const heave_port_fwd, const T *const heave_stbd_fwd,
                  const T *const heave_port_aft, const T *const heave_stbd_aft, T *residual) const
  {
    residual[0] = ((T(R_w2b.getRow(1).z()) * T(buoyancy) * T(-pos_buoyancy.z) +
                  T(R_w2b.getRow(2).z()) * T(buoyancy) * T(pos_buoyancy.y))*T(isBuoyant) +
                  sway_fwd[0] * T(-pos_sway_fwd.z) + sway_aft[0] * T(-pos_sway_aft.z) +
                  heave_port_fwd[0] * T(pos_heave_port_fwd.y) + heave_stbd_fwd[0] * T(pos_heave_stbd_fwd.y) +
                  heave_port_aft[0] * T(pos_heave_port_aft.y) + heave_stbd_aft[0] * T(pos_heave_stbd_aft.y) -
                  ((T(ang_v.z()) * T(ang_v.y())) * (T(Izz) - T(Iyy)))) / T(Ixx) -
                  T(cmdRoll);
    return true;
  }
};

// Pitch
// Thrusters contributing to a POSITIVE moment: heave_port_aft, heave_stbd_aft
// Thrusters contributting to a NEGATIVE moment: surge_port_lo, surge_stbd_lo, heave_port_fwd, heave_stbd_fwd
// Buoyancy X and Z components produce moments about y-axis
struct pitch
{
  template <typename T>
  bool operator()(const T *const surge_port_lo, const T *const surge_stbd_lo,
                  const T *const heave_port_fwd, const T *const heave_stbd_fwd,
                  const T *const heave_port_aft, const T *const heave_stbd_aft, T *residual) const
  {
    residual[0] = ((T(R_w2b.getRow(0).z()) * T(buoyancy) * T(pos_buoyancy.z) +
                  T(R_w2b.getRow(2).z()) * T(buoyancy) * T(-pos_buoyancy.x))*T(isBuoyant) +
                  surge_port_lo[0] * T(pos_surge_port_lo.z) + surge_stbd_lo[0] * T(pos_surge_stbd_lo.z) +
                  heave_port_fwd[0] * T(-pos_heave_port_fwd.x) + heave_stbd_fwd[0] * T(-pos_heave_stbd_fwd.x) +
                  heave_port_aft[0] * T(-pos_heave_port_aft.x) + heave_stbd_aft[0] * T(-pos_heave_stbd_aft.x) -
                  ((T(ang_v.x()) * T(ang_v.z())) * (T(Ixx) - T(Izz)))) / T(Iyy) -
                  T(cmdPitch);
    return true;
  }
};

// Yaw
// Thrusters contributing to a POSITIVE moment: surge_stbd_lo, sway_fwd
// Thrusters contributting to a NEGATIVE moment: surge_port_lo, sway_aft
// Buoyancy X and Y components produce moments about z-axis
struct yaw
{
  template <typename T>
  bool operator()(const T *const surge_port_lo, const T *const surge_stbd_lo,
                  const T *const sway_fwd, const T *const sway_aft, T *residual) const
  {
    residual[0] = ((T(R_w2b.getRow(0).z()) * T(buoyancy) * T(-pos_buoyancy.y) +
                  T(R_w2b.getRow(1).z()) * T(buoyancy) * T(pos_buoyancy.x))*T(isBuoyant) +
                  surge_port_lo[0] * T(-pos_surge_port_lo.y) + surge_stbd_lo[0] * T(-pos_surge_stbd_lo.y) +
                  sway_fwd[0] * T(pos_sway_fwd.x) + sway_aft[0] * T(pos_sway_aft.x) -
                  ((T(ang_v.y()) * T(ang_v.x())) * (T(Iyy) - T(Ixx)))) / T(Izz) -
                  T(cmdYaw);
    return true;
  }
};

// NOTE: It seems that ceres already tries to minimze all outputs as it solves.
// Hence, it seems unnecessary to add two more equations to create a
// SLE (system of linear eqns) composed of 8 equations and 8 unknowns)

/*** Tune Buoyancy ***/
// Purpose: Find the Center of Buoyancy (CoB)
// These equations ASSUME the vehicle is stationary in the water, attempting to
// reach a target orientation, but is unable to reach the said target because
// the moments due to buoyancy have not been factored into the angular eqns yet.
// The publised output will be the location of the CoB in relation to the CoM
// NOTE: Vehicle MUST be roughly stationary for output to make physical sense

// Tune Roll
// Thrusters contributing to a POSITIVE moment: sway_fwd, sway_aft, heave_port_fwd, heave_port_aft
// Thrusters contributting to a NEGATIVE moment: heave_stbd_fwd, heave_stbd_aft
// Buoyancy Y and Z components produce moments about x-axis
struct tuneRoll
{
  template <typename T>
  bool operator()(const T *const pos_buoyancy_y, const T *const pos_buoyancy_z, T *residual) const
  {
    residual[0] = T(R_w2b.getRow(1).z()) * T(buoyancy) * (-pos_buoyancy_z[0]) +
                  T(R_w2b.getRow(2).z()) * T(buoyancy) * pos_buoyancy_y[0] +
                  T(sway_fwd) * T(-pos_sway_fwd.z) + T(sway_aft) * T(-pos_sway_aft.z) +
                  T(heave_port_fwd) * T(pos_heave_port_fwd.y) + T(heave_port_aft) * T(pos_heave_port_aft.y) +
                  T(heave_stbd_fwd) * T(pos_heave_stbd_fwd.y) + T(heave_stbd_aft) * T(pos_heave_stbd_aft.y) -
                  (T(ang_v.z()) * T(ang_v.y())) * (T(Izz) - T(Iyy));
    return true;
  }
};

// Tune Pitch
// Thrusters contributing to a POSITIVE moment: heave_port_aft, heave_stbd_aft
// Thrusters contributting to a NEGATIVE moment: surge_port_lo, surge_stbd_lo, heave_port_fwd, heave_stbd_fwd
// Buoyancy X and Z components produce moments about y-axis
struct tunePitch
{
  template <typename T>
  bool operator()(const T *const pos_buoyancy_x, const T *const pos_buoyancy_z, T *residual) const
  {
    residual[0] = T(R_w2b.getRow(0).z()) * T(buoyancy) * pos_buoyancy_z[0] +
                  T(R_w2b.getRow(2).z()) * T(buoyancy) * (-pos_buoyancy_x[0]) +
                  T(surge_port_lo) * T(pos_surge_port_lo.z) + T(surge_stbd_lo) * T(pos_surge_stbd_lo.z) +
                  T(heave_port_aft) * T(-pos_heave_port_aft.x) + T(heave_stbd_aft) * T(-pos_heave_stbd_aft.x) +
                  T(heave_port_fwd) * T(-pos_heave_port_fwd.x) + T(heave_stbd_fwd) * T(-pos_heave_stbd_fwd.x) -
                  (T(ang_v.x()) * T(ang_v.z())) * (T(Ixx) - T(Izz));
    return true;
  }
};

// Tune Yaw
// Thrusters contributing to a POSITIVE moment: surge_stbd_lo, sway_fwd
// Thrusters contributting to a NEGATIVE moment: surge_port_lo, sway_aft
// Buoyancy X and Y components produce moments about z-axis
struct tuneYaw
{
  template <typename T>
  bool operator()(const T *const pos_buoyancy_x, const T *const pos_buoyancy_y, T *residual) const
  {
    residual[0] = T(R_w2b.getRow(0).z()) * T(buoyancy) * (-pos_buoyancy_y[0]) +
                  T(R_w2b.getRow(1).z()) * T(buoyancy) * (pos_buoyancy_x[0]) +
                  T(surge_port_lo) * T(-pos_surge_port_lo.y) + T(surge_stbd_lo) * T(-pos_surge_stbd_lo.y) +
                  T(sway_fwd) * T(pos_sway_fwd.x) + T(sway_aft) * T(pos_sway_aft.x) -
                  (T(ang_v.y()) * T(ang_v.x())) * (T(Iyy) - T(Ixx));
    return true;
  }
};
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thruster_controller");
  ThrusterController ThrusterController(argv);
  ThrusterController.Loop();
}

ThrusterController::ThrusterController(char **argv) : nh("thruster_controller") {
  // Load parameters from .yaml files or launch files
  nh.getParam("debug", debug_controller);
  ThrusterController::LoadParam("buoyancy_depth_thresh", buoyancy_depth_thresh); // Depth threshold to include buoyancy
  ThrusterController::LoadParam("buoyancy_pitch_thresh", buoyancy_pitch_thresh); // Pitch threshold to enable/disable heave thrusters

  // Load postions of each thruster relative to CoM
  ThrusterController::LoadParam<double>("HPF/X", pos_heave_port_fwd.x);
  ThrusterController::LoadParam<double>("HPF/Y", pos_heave_port_fwd.y);
  ThrusterController::LoadParam<double>("HPF/Z", pos_heave_port_fwd.z);

  ThrusterController::LoadParam<double>("HPA/X", pos_heave_port_aft.x);
  ThrusterController::LoadParam<double>("HPA/Y", pos_heave_port_aft.y);
  ThrusterController::LoadParam<double>("HPA/Z", pos_heave_port_aft.z);

  ThrusterController::LoadParam<double>("HSF/X", pos_heave_stbd_fwd.x);
  ThrusterController::LoadParam<double>("HSF/Y", pos_heave_stbd_fwd.y);
  ThrusterController::LoadParam<double>("HSF/Z", pos_heave_stbd_fwd.z);

  ThrusterController::LoadParam<double>("HSA/X", pos_heave_stbd_aft.x);
  ThrusterController::LoadParam<double>("HSA/Y", pos_heave_stbd_aft.y);
  ThrusterController::LoadParam<double>("HSA/Z", pos_heave_stbd_aft.z);

  ThrusterController::LoadParam<double>("SWF/X", pos_sway_fwd.x);
  ThrusterController::LoadParam<double>("SWF/Y", pos_sway_fwd.y);
  ThrusterController::LoadParam<double>("SWF/Z", pos_sway_fwd.z);

  ThrusterController::LoadParam<double>("SWA/X", pos_sway_aft.x);
  ThrusterController::LoadParam<double>("SWA/Y", pos_sway_aft.y);
  ThrusterController::LoadParam<double>("SWA/Z", pos_sway_aft.z);

  ThrusterController::LoadParam<double>("SPL/X", pos_surge_port_lo.x);
  ThrusterController::LoadParam<double>("SPL/Y", pos_surge_port_lo.y);
  ThrusterController::LoadParam<double>("SPL/Z", pos_surge_port_lo.z);

  ThrusterController::LoadParam<double>("SSL/X", pos_surge_stbd_lo.x);
  ThrusterController::LoadParam<double>("SSL/Y", pos_surge_stbd_lo.y);
  ThrusterController::LoadParam<double>("SSL/Z", pos_surge_stbd_lo.z);

  // Load vehicle properties
  ThrusterController::LoadParam<double>("Mass", mass);
  ThrusterController::LoadParam<double>("Volume", volume);
  ThrusterController::LoadParam<double>("Ixx", Ixx);
  ThrusterController::LoadParam<double>("Iyy", Iyy);
  ThrusterController::LoadParam<double>("Izz", Izz);
  ThrusterController::LoadParam<double>("Buoyancy_X_POS", pos_buoyancy.x);
  ThrusterController::LoadParam<double>("Buoyancy_Y_POS", pos_buoyancy.y);
  ThrusterController::LoadParam<double>("Buoyancy_Z_POS", pos_buoyancy.z);

  R_b2w.setIdentity();
  R_w2b.setIdentity();
  euler_deg.setZero();
  euler_rpy.setZero();
  ang_v.setZero();

  isBuoyant = false;
  enableHeaveFwd = true;
  enableHeaveAft = true;
  weight = mass*GRAVITY;
  buoyancy = volume*WATER_DENSITY*GRAVITY;

  state_sub = nh.subscribe<riptide_msgs::Imu>("/state/imu", 1, &ThrusterController::ImuCB, this);
  depth_sub = nh.subscribe<riptide_msgs::Depth>("/state/depth", 1, &ThrusterController::DepthCB, this);
  cmd_sub = nh.subscribe<geometry_msgs::Accel>("/command/accel", 1, &ThrusterController::AccelCB, this);
  cmd_pub = nh.advertise<riptide_msgs::ThrustStamped>("/command/thrust", 1);

  // Dynamic Reconfigure Variables
  cb = boost::bind(&ThrusterController::DynamicReconfigCallback, this, _1, _2);
  server.setCallback(cb);

  // Debug variables
  if(debug_controller) {
    buoyancy_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/debug/pos_buoyancy", 1);

    // Published in a message
    buoyancy_pos.vector.x = 0;
    buoyancy_pos.vector.y = 0;
    buoyancy_pos.vector.z = 0;
  }

  google::InitGoogleLogging(argv[0]);

  // PROBLEM SETUP
  // Add residual blocks (equations)

  // Linear
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<surge, 1, 1, 1>(new surge), NULL,
                           &surge_port_lo, &surge_stbd_lo);
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<sway, 1, 1, 1>(new sway), NULL,
                           &sway_fwd, &sway_aft);
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<heave, 1, 1, 1, 1, 1>(new heave), NULL,
                           &heave_port_fwd, &heave_stbd_fwd, &heave_port_aft, &heave_stbd_aft);

  // Angular
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<roll, 1, 1, 1, 1, 1, 1, 1>(new roll), NULL,
                           &sway_fwd, &sway_aft,
                           &heave_port_fwd, &heave_stbd_fwd, &heave_port_aft, &heave_stbd_aft);
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<pitch, 1, 1, 1, 1, 1, 1, 1>(new pitch), NULL,
                           &surge_port_lo, &surge_stbd_lo,
                           &heave_port_fwd, &heave_stbd_fwd, &heave_port_aft, &heave_stbd_aft);
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<yaw, 1, 1, 1, 1, 1>(new yaw), NULL,
                           &surge_port_lo, &surge_stbd_lo, &sway_fwd, &sway_aft);

  /*problem.SetParameterLowerBound(&surge_port_lo, 0, MIN_THRUST);
  problem.SetParameterUpperBound(&surge_port_lo, 0, MAX_THRUST);
  problem.SetParameterLowerBound(&surge_stbd_lo, 0, MIN_THRUST);
  problem.SetParameterUpperBound(&surge_stbd_lo, 0, MAX_THRUST);*/
  // Configure solver
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::DENSE_QR;

  //*** Tuning for Buoyancy ***////////////////////////////////////////////////
  buoyancyProblem.AddResidualBlock(new ceres::AutoDiffCostFunction<tuneRoll, 1, 1, 1>(new tuneRoll), NULL,
                           &pos_buoyancy_y, &pos_buoyancy_z);
  buoyancyProblem.AddResidualBlock(new ceres::AutoDiffCostFunction<tunePitch, 1, 1, 1>(new tunePitch), NULL,
                           &pos_buoyancy_x, &pos_buoyancy_z);
  buoyancyProblem.AddResidualBlock(new ceres::AutoDiffCostFunction<tuneYaw, 1, 1, 1>(new tuneYaw), NULL,
                           &pos_buoyancy_x, &pos_buoyancy_y);

  //buoyancyProblem.SetParameterLowerBound(&pos_buoyancy_x, 0, 0.001);

  // Configure solver
  buoyancyOptions.max_num_iterations = 50;
  buoyancyOptions.linear_solver_type = ceres::DENSE_QR;
  /////////////////////////////////////////////////////////////////////////////

#ifdef progress
  options.minimizer_progress_to_stdout = true;
#endif
}

// Load parameter from namespace
template <typename T>
void ThrusterController::LoadParam(std::string param, T &var)
{
  try
  {
    if (!nh.getParam(param, var))
    {
      throw 0;
    }
  }
  catch(int e)
  {
    std::string ns = nh.getNamespace();
    ROS_ERROR("Thruster Controller Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

// Callback for dynamic reconfigure
void ThrusterController::DynamicReconfigCallback(riptide_controllers::VehiclePropertiesConfig &config, uint32_t levels) {
  if(debug_controller) {
    mass = config.Mass;
    volume = config.Volume;
    pos_buoyancy.x = config.Buoyancy_X_POS;
    pos_buoyancy.y = config.Buoyancy_Y_POS;
    pos_buoyancy.z = config.Buoyancy_Z_POS;

    weight = mass*GRAVITY;
    buoyancy = volume*WATER_DENSITY*GRAVITY;
  }
}

//Get orientation from IMU
void ThrusterController::ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg)
{
  //Get euler angles, convert to radians, and make two rotation matrices
  vector3MsgToTF(imu_msg->euler_rpy, euler_deg);
  euler_rpy.setValue(euler_deg.x()*PI/180, euler_deg.y()*PI/180, euler_deg.z()*PI/180);
  R_b2w.setRPY(euler_rpy.x(), euler_rpy.y(), euler_rpy.z()); //Body to world rotations --> world_vector =  R_b2w * body_vector
  R_w2b = R_b2w.transpose(); //World to body rotations --> body_vector = R_w2b * world_vector

  //Get angular velocity and convert to [rad/s]
  vector3MsgToTF(imu_msg->ang_vel, ang_v);
  ang_v.setValue(ang_v.x()*PI/180, ang_v.y()*PI/180, ang_v.y()*PI/180);
}

//Get depth and determine if buoyancy should be included
void ThrusterController::DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg)
{
  if(depth_msg->depth > buoyancy_depth_thresh){
    isBuoyant = true;
    enableHeaveFwd = true;
    enableHeaveAft = true;
  }
  else {
    isBuoyant = false;

    // Enable/Disable Apprpriate Heave Thrusters
    if(euler_deg.y() > buoyancy_pitch_thresh) { // Aft is too high -> disable heave aft
      enableHeaveFwd = true;
      enableHeaveAft = false;
    }
    else if(euler_deg.y() < -buoyancy_pitch_thresh) { // Nose is too high -> disable heave fwd
      enableHeaveFwd = false;
      enableHeaveAft = true;
    }
    else { // Pitch within reasonable angle of operation
      enableHeaveFwd = true;
      enableHeaveAft = true;
    }
  }
}

void ThrusterController::AccelCB(const geometry_msgs::Accel::ConstPtr &a)
{
  cmdSurge = a->linear.x;
  cmdSway = a->linear.y;
  cmdHeave = a->linear.z;
  cmdRoll = a->angular.x;
  cmdPitch = a->angular.y;
  cmdYaw = a->angular.z;

  // These forced initial guesses don't make much of a difference.
  // We currently experience a sort of gimbal lock w/ or w/o them.
  surge_port_lo = 0.0;
  surge_stbd_lo = 0.0;
  sway_fwd = 0.0;
  sway_aft = 0.0;
  heave_port_aft = 0.0;
  heave_stbd_aft = 0.0;
  heave_stbd_fwd = 0.0;
  heave_port_fwd = 0.0;

  #ifdef debug
    std::cout << "Initial surge_port_lo = " << surge_port_lo << ", surge_stbd_lo = " << surge_stbd_lo
              << ", sway_fwd = " << sway_fwd << ", sway_aft = " << sway_aft << ", heave_port_aft = " << heave_port_aft
              << ", heave_stbd_aft = " << heave_stbd_aft << ", heave_stbd_fwd = " << heave_stbd_fwd
              << ", heave_port_fwd = " << heave_port_fwd << std::endl;
  #endif

  // Solve all my problems
  ceres::Solve(options, &problem, &summary);

  #ifdef report
    std::cout << summary.FullReport() << std::endl;
  #endif

  #ifdef debug
    std::cout << "Final surge_port_lo = " << surge_port_lo << ", surge_stbd_lo = " << surge_stbd_lo
              << ", sway_fwd = " << sway_fwd << ", sway_aft = " << sway_aft << ", heave_port_aft = " << heave_port_aft
              << ", heave_stbd_aft = " << heave_stbd_aft << ", heave_stbd_fwd = " << heave_stbd_fwd
              << ", heave_port_fwd = " << heave_port_fwd << std::endl;
  #endif

  //Forces are in POS dxn of the vehicle, where thrusts are what the
  //thruster outputs (POS thrust equals NEG vehicle dxn)
  thrust.header.stamp = ros::Time::now();
  thrust.force.surge_port_lo = -surge_port_lo;
  thrust.force.surge_stbd_lo = -surge_stbd_lo;
  thrust.force.sway_fwd = -sway_fwd;
  thrust.force.sway_aft = -sway_aft;
  thrust.force.heave_port_aft = -heave_port_aft;
  thrust.force.heave_stbd_aft = -heave_stbd_aft;
  thrust.force.heave_stbd_fwd = -heave_stbd_fwd;
  thrust.force.heave_port_fwd = -heave_port_fwd;

  cmd_pub.publish(thrust);

  // Tune Buoyancy - locate the center of buoyancy
  // The output will only make sense if the depth, roll, and pitch controllers
  // are initialized, and the vehicle is roughly stationary in the water.
  // The output should contain non-zero distances so long as the the vehicle is
  // unable to reach a target orientation along any axis.
  // The depth controller is used only to keep the vehicle fully submerged
  if(debug_controller) {
    // Initialize values
    pos_buoyancy_x = 0.0;
    pos_buoyancy_y = 0.0;
    pos_buoyancy_z = 0.0;

    ceres::Solve(buoyancyOptions, &buoyancyProblem, &buoyancySummary);
    buoyancy_pos.header.stamp = ros::Time::now();
    buoyancy_pos.vector.x = pos_buoyancy_x;
    buoyancy_pos.vector.y = pos_buoyancy_y;
    buoyancy_pos.vector.z = pos_buoyancy_z;

    buoyancy_pub.publish(buoyancy_pos);
  }
}

void ThrusterController::Loop()
{
  ros::Rate rate(200);
  while(!ros::isShuttingDown()) {
    ros::spinOnce();
    rate.sleep();
  }
}
