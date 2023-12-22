import numpy as np
from common.realtime import sec_since_boot, DT_MDL
from common.numpy_fast import interp
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc
from selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import N as LAT_MPC_N
from selfdrive.controls.lib.drive_helpers import CONTROL_N, MIN_SPEED
from selfdrive.controls.lib.lane_planner import LanePlanner, TRAJECTORY_SIZE
from selfdrive.controls.lib.desire_helper import DesireHelper#, AUTO_LCA_START_TIME
import cereal.messaging as messaging
from common.params import Params
from decimal import Decimal
from cereal import log

LaneChangeState = log.LateralPlan.LaneChangeState

PATH_COST = 1.0
LATERAL_MOTION_COST = 0.11
LATERAL_ACCEL_COST = 0.0
LATERAL_JERK_COST = 0.04
# Extreme steering rate is unpleasant, even
# when it does not cause bad jerk.
# TODO this cost should be lowered when low
# speed lateral control is stable on all cars
STEERING_RATE_COST = 700.0



class LateralPlanner:
  def __init__(self, CP, use_lanelines=True, wide_camera=False):
    self.use_lanelines = use_lanelines
    self.LP = LanePlanner(wide_camera)
    self.readParams = 0
    self.DH = DesireHelper()

     # Vehicle model parameters used to calculate lateral movement of car
    self.factor1 = CP.wheelbase - CP.centerToFront
    self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)
    
    self.pathOffset = float(int(Params().get("PathOffset", encoding="utf8")))*0.01
    self.last_cloudlog_t = 0
    self.solution_invalid_cnt = 0

    self.path_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.velocity_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.path_xyz_stds = np.ones((TRAJECTORY_SIZE, 3))
    self.plan_yaw = np.zeros((TRAJECTORY_SIZE,))
    self.plan_yaw_rate = np.zeros((TRAJECTORY_SIZE,))
    self.t_idxs = np.arange(TRAJECTORY_SIZE)
    self.y_pts = np.zeros((TRAJECTORY_SIZE,))
    self.v_plan = np.zeros((TRAJECTORY_SIZE,))
    self.v_ego = 0.0
    self.l_lane_change_prob = 0.0
    self.r_lane_change_prob = 0.0
    self.d_path_w_lines_xyz = np.zeros((TRAJECTORY_SIZE, 3))

    self.lat_mpc = LateralMpc()
    self.reset_mpc(np.zeros(4))
    
    self.dynamic_lane_profile = int(Params().get("DynamicLaneProfile", encoding="utf8"))
    self.dynamic_lane_profile_status = False
    self.dynamic_lane_profile_status_buffer = False
    self.second = 0.0

    self.average_desired_curvature = CP.pfeiferjDesiredCurvatures
    
  def reset_mpc(self, x0=np.zeros(4)):
    self.x0 = x0
    self.lat_mpc.reset(x0=self.x0)

  def update(self, sm):
    try:
      if CP.lateralTuning.which() == 'pid':
        self.output_scale = sm['controlsState'].lateralControlState.pidState.output
      elif CP.lateralTuning.which() == 'indi':
        self.output_scale = sm['controlsState'].lateralControlState.indiState.output
      elif CP.lateralTuning.which() == 'lqr':
        self.output_scale = sm['controlsState'].lateralControlState.lqrState.output
      elif CP.lateralTuning.which() == 'torque':
        self.output_scale = sm['controlsState'].lateralControlState.torqueState.output  
    except:
      pass
    self.readParams -= 1
    if self.readParams <= 0:
      self.readParams = 100
    self.second += DT_MDL
    if self.second > 1.0:
      self.use_lanelines = not Params().get_bool("EndToEndToggle")
      self.dynamic_lane_profile = int(Params().get("DynamicLaneProfile", encoding="utf8"))
      self.second = 0.0
    elif self.readParams == 50:
      self.pathOffset = float(int(Params().get("PathOffset", encoding="utf8")))*0.01
      
    # clip speed , lateral planning is not possible at 0 speed
    measured_curvature = sm['controlsState'].curvature

    # Parse model predictions
    md = sm['modelV2']
    self.LP.parse_model(md)
    if len(md.position.x) == TRAJECTORY_SIZE and len(md.orientation.x) == TRAJECTORY_SIZE:
      self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
      self.t_idxs = np.array(md.position.t)
      self.plan_yaw = np.array(md.orientation.z)
      self.plan_yaw_rate = np.array(md.orientationRate.z)
      self.velocity_xyz = np.column_stack([md.velocity.x, md.velocity.y, md.velocity.z])
      car_speed = np.linalg.norm(self.velocity_xyz, axis=1)
      self.v_plan = np.clip(car_speed, MIN_SPEED, np.inf)
      self.v_ego = self.v_plan[0]
    if len(md.position.xStd) == TRAJECTORY_SIZE:
      self.path_xyz_stds = np.column_stack([md.position.xStd, md.position.yStd, md.position.zStd])

    # Lane change logic
    lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob
    self.DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob, md)

    # Turn off lanes during lane change
    if self.DH.desire == log.LateralPlan.Desire.laneChangeRight or self.DH.desire == log.LateralPlan.Desire.laneChangeLeft:
      self.LP.lll_prob *= self.DH.lane_change_ll_prob
      self.LP.rll_prob *= self.DH.lane_change_ll_prob

    # Calculate final driving path and set MPC costs
    if self.use_lanelines:
      d_path_xyz = self.LP.get_d_path(self.v_ego, self.t_idxs, self.path_xyz)
      d_path_xyz[:, 1] += self.pathOffset
      self.lat_mpc.set_weights(PATH_COST, LATERAL_MOTION_COST,
                               LATERAL_ACCEL_COST, LATERAL_JERK_COST,
                               STEERING_RATE_COST)
      self.dynamic_lane_profile_status = False
    elif self.dynamic_lane_profile == 0:
      d_path_xyz = self.LP.get_d_path(self.v_ego, self.t_idxs)
      d_path_xyz[:, 1] += self.pathOffset
      self.lat_mpc.set_weights(PATH_COST, LATERAL_MOTION_COST,
                               LATERAL_ACCEL_COST, LATERAL_JERK_COST,
                               STEERING_RATE_COST)
      self.dynamic_lane_profile_status = False
    elif self.dynamic_lane_profile == 1:
      d_path_xyz[:, 1] += self.pathOffset
      self.lat_mpc.set_weights(PATH_COST, LATERAL_MOTION_COST,
                               LATERAL_ACCEL_COST, LATERAL_JERK_COST,
                               STEERING_RATE_COST)
      self.dynamic_lane_profile_status = True
    elif self.dynamic_lane_profile == 2 and ((self.LP.lll_prob + self.LP.rll_prob)/2 < 0.3) and self.DH.lane_change_state == LaneChangeState.off:
      d_path_xyz[:, 1] += self.pathOffset
      self.lat_mpc.set_weights(PATH_COST, LATERAL_MOTION_COST,
                               LATERAL_ACCEL_COST, LATERAL_JERK_COST,
                               STEERING_RATE_COST)
      self.dynamic_lane_profile_status = True
      self.dynamic_lane_profile_status_buffer = True
    elif self.dynamic_lane_profile == 2 and ((self.LP.lll_prob + self.LP.rll_prob)/2 > 0.5) and \
     self.dynamic_lane_profile_status_buffer and self.DH.lane_change_state == LaneChangeState.off:
      d_path_xyz = self.LP.get_d_path(self.v_ego, self.t_idxs)
      d_path_xyz[:, 1] += self.pathOffset
      self.lat_mpc.set_weights(PATH_COST, LATERAL_MOTION_COST,
                               LATERAL_ACCEL_COST, LATERAL_JERK_COST,
                               STEERING_RATE_COST)
      self.dynamic_lane_profile_status = False
      self.dynamic_lane_profile_status_buffer = False
    elif self.dynamic_lane_profile == 2 and self.dynamic_lane_profile_status_buffer == True and self.DH.lane_change_state == LaneChangeState.off:
      d_path_xyz[:, 1] += self.pathOffset
      self.lat_mpc.set_weights(PATH_COST, LATERAL_MOTION_COST,
                               LATERAL_ACCEL_COST, LATERAL_JERK_COST,
                               STEERING_RATE_COST)
      self.dynamic_lane_profile_status = True
    else:
      d_path_xyz = self.LP.get_d_path(self.v_ego, self.t_idxs)
      d_path_xyz[:, 1] += self.pathOffset
      self.lat_mpc.set_weights(PATH_COST, LATERAL_MOTION_COST,
                               LATERAL_ACCEL_COST, LATERAL_JERK_COST,
                               STEERING_RATE_COST)
      self.dynamic_lane_profile_status = False
      self.dynamic_lane_profile_status_buffer = False
    
    y_pts = self.path_xyz[:LAT_MPC_N+1, 1]
    heading_pts = self.plan_yaw[:LAT_MPC_N+1]
    yaw_rate_pts = self.plan_yaw_rate[:LAT_MPC_N+1]
    self.y_pts = y_pts
    
    assert len(y_pts) == LAT_MPC_N + 1
    assert len(heading_pts) == LAT_MPC_N + 1
    assert len(yaw_rate_pts) == LAT_MPC_N + 1
    lateral_factor = np.clip(self.factor1 - (self.factor2 * self.v_plan**2), 0.0, np.inf)
    p = np.column_stack([self.v_plan, lateral_factor])
    self.lat_mpc.run(self.x0,
                     p,
                     y_pts,
                     heading_pts,
                     yaw_rate_pts)
    # init state for next
    self.x0[3] = interp(DT_MDL, self.t_idxs[:LAT_MPC_N + 1], self.lat_mpc.x_sol[:, 3])

    #  Check for infeasible MPC solution
    mpc_nans = np.isnan(self.lat_mpc.x_sol[:, 3]).any()
    t = sec_since_boot()
    if mpc_nans or self.lat_mpc.solution_status != 0:
      self.reset_mpc()
      self.x0[3] = measured_curvature * self.v_ego
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Lateral mpc - nan: True")

    if self.lat_mpc.cost > 1e6 or mpc_nans:
      self.solution_invalid_cnt += 1
    else:
      self.solution_invalid_cnt = 0

  def publish(self, sm, pm):
    plan_solution_valid = self.solution_invalid_cnt < 2
    plan_send = messaging.new_message('lateralPlan')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'modelV2'])

    lateralPlan = plan_send.lateralPlan
    lateralPlan.modelMonoTime = sm.logMonoTime['modelV2']
    lateralPlan.laneWidth = float(self.LP.lane_width)
    lateralPlan.dPathPoints = self.y_pts.tolist()
    lateralPlan.psis = self.lat_mpc.x_sol[0:CONTROL_N, 2].tolist()

    lateralPlan.curvatures = (self.lat_mpc.x_sol[0:CONTROL_N, 3]/self.v_ego).tolist()
    lateralPlan.curvatureRates = [float(x/self.v_ego) for x in self.lat_mpc.u_sol[0:CONTROL_N - 1]] + [0.0]
    
    lateralPlan.lProb = float(self.LP.lll_prob)
    lateralPlan.rProb = float(self.LP.rll_prob)
    lateralPlan.dProb = float(self.LP.d_prob)

    lateralPlan.mpcSolutionValid = bool(plan_solution_valid)
    lateralPlan.solverExecutionTime = self.lat_mpc.solve_time

    lateralPlan.desire = self.DH.desire
    lateralPlan.useLaneLines = self.use_lanelines
    lateralPlan.laneChangeState = self.DH.lane_change_state
    lateralPlan.laneChangeDirection = self.DH.lane_change_direction
    
    #lateralPlan.autoLaneChangeEnabled = self.DH.auto_lane_change_enabled
    #lateralPlan.autoLaneChangeTimer = int(AUTO_LCA_START_TIME) - int(self.DH.auto_lane_change_timer)
    
    plan_send.lateralPlan.dPathWLinesX = [float(x) for x in self.d_path_w_lines_xyz[:, 0]]
    plan_send.lateralPlan.dPathWLinesY = [float(y) for y in self.d_path_w_lines_xyz[:, 1]]
    
    lateralPlan.dynamicLaneProfile = bool(self.dynamic_lane_profile_status)
    
    lateralPlan.totalCameraOffset = float(self.LP.total_camera_offset)
    
    pm.send('lateralPlan', plan_send)
