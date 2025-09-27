package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.mw_lib.subsystem.MWSubsystem;
import frc.mw_lib.swerve_lib.ChassisRequest;
import frc.mw_lib.swerve_lib.ChassisRequest.XPositiveReference;
import frc.mw_lib.swerve_lib.module.Module.DriveControlMode;
import frc.mw_lib.swerve_lib.module.Module.SteerControlMode;
import frc.robot.Constants;
import frc.robot.OI;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Swerve extends MWSubsystem<SwerveIO, frc.robot.subsystems.swerve.Swerve.SwerveStates> {

  // Current system states for the swerve drive
  enum SwerveStates {
    FIELD_CENTRIC,
    ROBOT_CENTRIC,
    CHOREO_PATH,
    ROTATION_LOCK,
    TRACTOR_BEAM,
    IDLE
  }

  private static Swerve instance_ = null;

  public static Swerve getInstance() {
    if (instance_ == null) {
      if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
        instance_ = new Swerve(new SwerveIOReal());
      } else if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
        instance_ = new Swerve(new SwerveIOSim());
      } else {
        instance_ = new Swerve(new SwerveIO() {});
      }
    }
    return instance_;
  }

  // State Specific Members
  Trajectory<SwerveSample> desired_choreo_traj_;
  private final Timer choreo_timer_ = new Timer();
  private Optional<SwerveSample> choreo_sample_to_apply_;
  private final PIDController choreo_x_controller_ =
      new PIDController(
          SwerveConstants.CHOREO_X_CONTROLLER_KP,
          SwerveConstants.CHOREO_X_CONTROLLER_KI,
          SwerveConstants.CHOREO_X_CONTROLLER_KD);
  private final PIDController choreo_y_controller_ =
      new PIDController(
          SwerveConstants.CHOREO_Y_CONTROLLER_KP,
          SwerveConstants.CHOREO_Y_CONTROLLER_KI,
          SwerveConstants.CHOREO_Y_CONTROLLER_KD);
  private final PIDController choreo_theta_controller_ =
      new PIDController(
          SwerveConstants.CHOREO_THETA_CONTROLLER_KP,
          SwerveConstants.CHOREO_THETA_CONTROLLER_KI,
          SwerveConstants.CHOREO_THETA_CONTROLLER_KD);
  private Pose2d desired_tractor_beam_pose_ = new Pose2d();
  private double max_lin_vel_for_tractor_beam_;
  private double max_ang_vel_for_tractor_beam_;
  private final PIDController tractor_beam_controller_ =
      new PIDController(
          SwerveConstants.TRACTOR_BEAM_CONTROLLER_KP,
          SwerveConstants.TRACTOR_BEAM_CONTROLLER_KI,
          SwerveConstants.TRACTOR_BEAM_CONTROLLER_KD);
  private Rotation2d desired_rotation_lock_rot_ = new Rotation2d();
  private double tele_op_velocity_scalar_ = 1.0;

  // IO Members
  static final Lock odometry_lock_ = new ReentrantLock();
  private SwerveIO swerve_io_;

  private Rotation2d operator_forward_direction_ =
      SwerveConstants.OperatorPerspective.BLUE_ALLIANCE.heading;

  private ChassisRequest.FieldCentric field_centric_request_;
  private ChassisRequest.RobotCentric robot_centric_request_;
  private ChassisRequest.FieldCentricFacingAngle rotation_lock_request_;
  private ChassisRequest.ApplyFieldSpeeds field_speeds_request_;

  /**
   * Creates a new Swerve subsystem with the specified gyro and module IOs.
   *
   * @param io The swerve I/O container to use
   */
  public Swerve(SwerveIO io) {
    super(SwerveStates.IDLE);
    this.io = io;

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Initialize drive mode requests
    field_centric_request_ =
        new ChassisRequest.FieldCentric()
            .withDriveRequestType(DriveControlMode.OPEN_LOOP)
            .withSteerRequestType(SteerControlMode.CLOSED_LOOP)
            .withDeadband(SwerveConstants.MAX_TRANSLATION_RATE * 0.01)
            .withRotationalDeadband(SwerveConstants.MAX_ANGULAR_RATE * 0.01)
            .withXPositiveReference(XPositiveReference.OperatorPerspective);
    robot_centric_request_ =
        new ChassisRequest.RobotCentric()
            .withDriveRequestType(DriveControlMode.OPEN_LOOP)
            .withSteerRequestType(SteerControlMode.CLOSED_LOOP)
            .withDeadband(SwerveConstants.MAX_TRANSLATION_RATE * 0.01)
            .withRotationalDeadband(SwerveConstants.MAX_ANGULAR_RATE * 0.01);
    rotation_lock_request_ =
        new ChassisRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveControlMode.OPEN_LOOP)
            .withSteerRequestType(SteerControlMode.CLOSED_LOOP)
            .withDeadband(SwerveConstants.MAX_TRANSLATION_RATE * 0.01)
            .withRotationalDeadband(SwerveConstants.MAX_ANGULAR_RATE * 0.01)
            .withXPositiveReference(XPositiveReference.OperatorPerspective);
    field_speeds_request_ =
        new ChassisRequest.ApplyFieldSpeeds()
            .withDriveRequestType(DriveControlMode.CLOSED_LOOP)
            .withSteerRequestType(SteerControlMode.CLOSED_LOOP);
  }

  @Override
  public void updateLogic(double timestamp) {
    // Update the request to apply based on the system state
    switch (system_state_) {
      case FIELD_CENTRIC:
        io.current_request =
            field_centric_request_.withTwist(calculateSpeedsBasedOnJoystickInputs());
        break;
      case ROBOT_CENTRIC:
        io.current_request =
            robot_centric_request_.withTwist(calculateSpeedsBasedOnJoystickInputs());
        break;
      case TRACTOR_BEAM:
        tractorBeamState();
        break;
      case ROTATION_LOCK:
        io.current_request = rotation_lock_request_.withTargetHeading(desired_rotation_lock_rot_);
        break;
      case CHOREO_PATH:
        choreoPathState();
        break;
      case IDLE:
      default:
        io.current_request = new ChassisRequest.Idle();
        break;
    }

    // Set state static request parameters
    io.current_request_parameters.currentChassisSpeed = getChassisSpeeds();
    io.current_request_parameters.currentPose = getPose();
    io.current_request_parameters.updatePeriod =
        Timer.getFPGATimestamp() - io.current_request_parameters.timestamp;
    io.current_request_parameters.timestamp = Timer.getFPGATimestamp();
    io.current_request_parameters.operatorForwardDirection = operator_forward_direction_;

    DogLog.log(getSubsystemKey() + "/ModuleStates", getModuleStates());
    DogLog.log(getSubsystemKey() + "/ChassisSpeeds", getChassisSpeeds());
    DogLog.log(getSubsystemKey() + "/Rotation", getRotation());
    DogLog.log(getSubsystemKey() + "/Pose", getPose());
    DogLog.log(getSubsystemKey() + "/Raw Gyro", getGyroRotation());
  }

  @Override
  public void reset() {}

  // ------------------------------------------------
  // Subsystem State Update Methods
  // ------------------------------------------------

  /**
   * Gets the current system state of the Swerve subsystem.
   *
   * @return the current system state
   */
  @Override
  protected void handleStateTransition(SwerveStates wanted_state) {
    system_state_ =
        switch (wanted_state) {
          case FIELD_CENTRIC -> SwerveStates.FIELD_CENTRIC;
          case ROBOT_CENTRIC -> SwerveStates.ROBOT_CENTRIC;
          case CHOREO_PATH -> {
            if (system_state_ != SwerveStates.CHOREO_PATH) {
              choreo_timer_.restart();
              choreo_sample_to_apply_ =
                  desired_choreo_traj_.sampleAt(
                      choreo_timer_.get(), SwerveConstants.FLIP_TRAJECTORY_ON_RED);
              yield SwerveStates.CHOREO_PATH;
            } else {
              choreo_sample_to_apply_ =
                  desired_choreo_traj_.sampleAt(
                      choreo_timer_.get(), SwerveConstants.FLIP_TRAJECTORY_ON_RED);
              yield SwerveStates.CHOREO_PATH;
            }
          }
          case ROTATION_LOCK -> SwerveStates.ROTATION_LOCK;
          case TRACTOR_BEAM -> SwerveStates.TRACTOR_BEAM;
          default -> SwerveStates.IDLE;
        };
  }

  /**
   * Handles the TRACTOR_BEAM state by calculating the necessary chassis speeds to move towards the
   * desired tractor beam pose.
   */
  private void tractorBeamState() {
    Translation2d translation_to_desired_point =
        desired_tractor_beam_pose_.getTranslation().minus(getPose().getTranslation());
    double linear_distance = translation_to_desired_point.getNorm();
    double friction_constant = 0.0;
    if (linear_distance >= Units.inchesToMeters(0.5)) {
      friction_constant =
          SwerveConstants.TRACTOR_BEAM_STATIC_FRICTION_CONSTANT
              * SwerveConstants.MAX_TRANSLATION_RATE;
    }
    Rotation2d direction_of_travel = translation_to_desired_point.getAngle();
    double velocity_output =
        Math.min(
            Math.abs(tractor_beam_controller_.calculate(linear_distance, 0)) + friction_constant,
            max_lin_vel_for_tractor_beam_);
    double x_component = velocity_output * direction_of_travel.getCos();
    double y_component = velocity_output * direction_of_travel.getSin();

    DogLog.log(getSubsystemKey() + "TractorBeam/X Velocity Component", x_component);
    DogLog.log(getSubsystemKey() + "TractorBeam/Y Velocity Component", y_component);
    DogLog.log(getSubsystemKey() + "TractorBeam/Velocity Output", velocity_output);
    DogLog.log(getSubsystemKey() + "TractorBeam/Linear Distance", linear_distance);
    DogLog.log(getSubsystemKey() + "TractorBeam/Direction of Travel", direction_of_travel);
    DogLog.log(getSubsystemKey() + "TractorBeam/Desired Point", desired_tractor_beam_pose_);

    if (Double.isNaN(max_ang_vel_for_tractor_beam_)) {
      io.current_request =
          rotation_lock_request_
              .withTwist(new Twist2d(x_component, y_component, 0.0))
              .withTargetHeading(desired_tractor_beam_pose_.getRotation());
    } else {
      io.current_request =
          rotation_lock_request_
              .withTwist(new Twist2d(x_component, y_component, 0.0))
              .withTargetHeading(desired_tractor_beam_pose_.getRotation())
              .withMaxAbsRotationalRate(max_ang_vel_for_tractor_beam_);
    }
  }

  /**
   * Handles the CHOREO_PATH state by applying the appropriate chassis speeds based on the current
   * trajectory sample and PID controller outputs.
   */
  private void choreoPathState() {
    if (choreo_sample_to_apply_.isPresent()) {
      SwerveSample sample = choreo_sample_to_apply_.get();
      DogLog.log(getSubsystemKey() + "/Choreo/Timer Value", choreo_timer_.get());
      DogLog.log(getSubsystemKey() + "/Choreo/Traj Name", desired_choreo_traj_.name());
      DogLog.log(getSubsystemKey() + "/Choreo/Total time", desired_choreo_traj_.getTotalTime());
      DogLog.log(getSubsystemKey() + "/Choreo/sample/Desired Pose", sample.getPose());
      DogLog.log(
          getSubsystemKey() + "/Choreo/sample/Desired Chassis Speeds", sample.getChassisSpeeds());
      DogLog.log(getSubsystemKey() + "/Choreo/sample/Module Forces X", sample.moduleForcesX());
      DogLog.log(getSubsystemKey() + "/Choreo/sample/Module Forces Y", sample.moduleForcesY());
      Pose2d pose = getPose();
      ChassisSpeeds target_speeds = sample.getChassisSpeeds();
      target_speeds.vxMetersPerSecond += choreo_x_controller_.calculate(pose.getX(), sample.x);
      target_speeds.vyMetersPerSecond += choreo_y_controller_.calculate(pose.getY(), sample.y);
      target_speeds.omegaRadiansPerSecond +=
          choreo_theta_controller_.calculate(pose.getRotation().getRadians(), sample.heading);

      io.current_request = field_speeds_request_.withSpeeds(target_speeds);
    } else {
      // If no sample is available, we will just stop the robot
      io.current_request = new ChassisRequest.Idle();
    }
  }

  // ------------------------------------------------
  // Chassis Control Methods
  // ------------------------------------------------

  /**
   * Updates the internal target for the robot to follow and begins CHOREO_PATH
   *
   * @param trajectory the trajectory for the robot to follow
   */
  public void setDesiredChoreoTrajectory(Trajectory<SwerveSample> trajectory) {
    desired_choreo_traj_ = trajectory;
  }

  /**
   * Updates the internal target for the robot to reach and begins TRACTOR_BEAM
   *
   * @param target_pose target pose for the robot to reach
   */
  public void setDesiredTractorBeamPose(Pose2d pose) {
    desired_tractor_beam_pose_ = pose;
    max_lin_vel_for_tractor_beam_ = Double.NaN;
    max_ang_vel_for_tractor_beam_ = Double.NaN;
  }

  /**
   * Updates the internal target for the robot to reach and begins TRACTOR_BEAM
   *
   * @param pose target pose for the robot to reach
   * @param max_lin_vel maximum linear velocity for the robot to reach the target pose
   */
  public void setDesiredTractorBeamPoseWithMaxLinVel(Pose2d pose, double max_lin_vel) {
    max_lin_vel_for_tractor_beam_ = max_lin_vel;
    max_lin_vel_for_tractor_beam_ = Double.NaN;
    desired_tractor_beam_pose_ = pose;
  }

  /**
   * Updates the internal target for the robot to reach and begins TRACTOR_BEAM
   *
   * @param pose target pose for the robot to reach
   * @param max_ang_vel maximum angular velocity for the robot to reach the target pose
   */
  public void setDesiredTractorBeamPoseWithMaxAngVel(Pose2d pose, double max_ang_vel) {
    max_lin_vel_for_tractor_beam_ = max_ang_vel;
    max_lin_vel_for_tractor_beam_ = Double.NaN;
    desired_tractor_beam_pose_ = pose;
  }

  /**
   * Updates the internal target for the robot to reach and begins TRACTOR_BEAM
   *
   * @param pose target pose for the robot to reach
   * @param max_lin_vel maximum linear velocity for the robot to reach the target pose
   * @param max_ang_vel maximum angular velocity for the robot to reach the target pose
   */
  public void setTractorBeamPoseWithConstraints(
      Pose2d pose, double max_lin_vel, double max_ang_vel) {
    max_lin_vel_for_tractor_beam_ = max_lin_vel;
    max_ang_vel_for_tractor_beam_ = max_ang_vel;
    desired_tractor_beam_pose_ = pose;
  }

  /**
   * Updates the internal target for the robot to face and begins ROTATION_LOCK
   *
   * @param rotation
   */
  public void setDesiredRotationLock(Rotation2d rotation) {
    desired_rotation_lock_rot_ = rotation;
  }

  public Command toggleFieldCentric() {
    return Commands.runOnce(
        () -> {
          if (system_state_ == SwerveStates.FIELD_CENTRIC) {
            setWantedState(SwerveStates.ROBOT_CENTRIC);
          } else {
            setWantedState(SwerveStates.FIELD_CENTRIC);
          }
        });
  }

  // ------------------------------------------------
  // Operator Interface Methods
  // ------------------------------------------------

  /**
   * Calculates chassis speeds based on joystick inputs.
   *
   * @return the controller inputs as a Twist2d object, where the x and y components represent the
   *     translation speeds and the theta component represents the angular speed
   */
  private Twist2d calculateSpeedsBasedOnJoystickInputs() {
    if (DriverStation.getAlliance().isEmpty()) {
      return new Twist2d();
    }

    double x_magnitude =
        -MathUtil.applyDeadband(OI.getDriverJoystickLeftY(), Constants.CONTROLLER_DEADBAND);
    double y_magnitude =
        -MathUtil.applyDeadband(OI.getDriverJoystickLeftX(), Constants.CONTROLLER_DEADBAND);
    double angular_magnitude =
        -MathUtil.applyDeadband(OI.getDriverJoystickRightX(), Constants.CONTROLLER_DEADBAND);

    Twist2d twist =
        new Twist2d(
            x_magnitude * SwerveConstants.MAX_TRANSLATION_RATE * tele_op_velocity_scalar_,
            y_magnitude * SwerveConstants.MAX_TRANSLATION_RATE * tele_op_velocity_scalar_,
            angular_magnitude * SwerveConstants.MAX_ANGULAR_RATE);
    return twist;
  }

  /**
   * Sets the operator forward direction based on the operator's perspective.
   *
   * @param reference the operator's perspective reference
   */
  public void setOperatorForwardDirection(SwerveConstants.OperatorPerspective reference) {
    operator_forward_direction_ = reference.heading;
  }

  /**
   * Sets the teleop velocity scalar to scale the robot's speed.
   *
   * @param scalar the scalar value to set, clamped between 0 and 1
   */
  public void setTeleOpVelocityScalar(double scalar) {
    tele_op_velocity_scalar_ = MathUtil.clamp(scalar, 0, 1);
  }

  // ------------------------------------------------
  // Status Check Methods
  // ------------------------------------------------

  /**
   * Checks if the robot is at the tractor beam setpoint.
   *
   * @return true if the robot is at the tractor beam setpoint, false otherwise
   */
  public boolean isAtTractorBeamSetpoint() {
    double distance =
        desired_tractor_beam_pose_.getTranslation().minus(getPose().getTranslation()).getNorm();
    return MathUtil.isNear(0.0, distance, SwerveConstants.TRACTOR_BEAM_TRANSLATION_ERROR_MARGIN);
  }

  /**
   * Checks if the robot is at the desired rotation.
   *
   * @return true if the robot is at the desired rotation, false otherwise
   */
  public boolean isAtDesiredRotation() {
    return isAtDesiredRotation(Units.degreesToRadians(10.0));
  }

  /**
   * Checks if the robot is at the desired rotation within a specified tolerance.
   *
   * @param tolerance the tolerance in radians
   * @return true if the robot is at the desired rotation within the tolerance, false otherwise
   */
  public boolean isAtDesiredRotation(double tolerance) {
    return rotation_lock_request_.HeadingController.getPositionError() < tolerance;
  }

  /**
   * Checks if the robot is at the choreo setpoint.
   *
   * @return true if the robot is at the choreo setpoint, false otherwise
   */
  public boolean isAtChoreoSetpoint() {
    if (system_state_ != SwerveStates.CHOREO_PATH) {
      return false;
    }
    return MathUtil.isNear(
            desired_choreo_traj_.getFinalPose(true).get().getX(),
            getPose().getX(),
            SwerveConstants.CHOREO_TRANSLATION_ERROR_MARGIN)
        && MathUtil.isNear(
            desired_choreo_traj_.getFinalPose(true).get().getY(),
            getPose().getY(),
            SwerveConstants.CHOREO_TRANSLATION_ERROR_MARGIN);
  }

  /**
   * Checks if the robot is at the end of the choreo trajectory or at the tractor beam setpoint.
   *
   * @return true if the robot is at the end of the choreo trajectory or at the tractor beam
   *     setpoint, false otherwise
   */
  public boolean isAtEndOfChoreoTrajectoryOrTractorBeam() {
    if (desired_choreo_traj_ != null) {
      return (MathUtil.isNear(
                  desired_choreo_traj_.getFinalPose(true).get().getX(),
                  getPose().getX(),
                  SwerveConstants.CHOREO_TRANSLATION_ERROR_MARGIN))
              && MathUtil.isNear(
                  desired_choreo_traj_.getFinalPose(true).get().getY(),
                  getPose().getY(),
                  SwerveConstants.CHOREO_TRANSLATION_ERROR_MARGIN)
          || isAtTractorBeamSetpoint();
    } else {
      return isAtTractorBeamSetpoint();
    }
  }

  /**
   * Gets the distance from the choreo endpoint.
   *
   * @return the distance from the choreo endpoint in meters
   */
  public double getDistanceFromChoreoEndpoint() {
    double distance =
        Math.abs(
            desired_choreo_traj_
                .getFinalPose(SwerveConstants.FLIP_TRAJECTORY_ON_RED)
                .get()
                .minus(getPose())
                .getTranslation()
                .getNorm());
    return distance;
  }

  /**
   * Gets the distance from the tractor beam setpoint.
   *
   * @return the distance from the tractor beam setpoint in meters
   */
  public double getDistanceFromTractorBeamSetpoint() {
    double diff =
        desired_tractor_beam_pose_.getTranslation().minus(getPose().getTranslation()).getNorm();
    return diff;
  }

  // ------------------------------------------------
  // Chassis Property Methods
  // ------------------------------------------------

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  public SwerveModuleState[] getModuleStates() {
    return io.module_states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  public SwerveModulePosition[] getModulePositions() {
    return io.module_positions;
  }

  /** Returns the measured chassis speeds of the robot. */
  public ChassisSpeeds getChassisSpeeds() {
    return io.chassis_speeds;
  }

  /** Returns the current odometry pose */
  public Pose2d getPose() {
    return io.pose;
  }

  /** Returns the current odometry rotation */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * Updates the current pose for the pose estimator
   *
   * @param pose
   */
  public void setPose(Pose2d pose) {
    swerve_io_.resetPose(pose);
  }

  /** Returns the raw gyro rotation */
  public Rotation2d getGyroRotation() {
    return io.raw_gyro_rotation;
  }
}
