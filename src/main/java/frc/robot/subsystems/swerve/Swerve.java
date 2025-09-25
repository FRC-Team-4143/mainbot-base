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
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.mw_lib.subsystem.MWSubsystem;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.swerve.ChassisRequest.XPositiveReference;
import frc.robot.subsystems.swerve.module.Module.DriveControlMode;
import frc.robot.subsystems.swerve.module.Module.SteerControlMode;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Swerve extends MWSubsystem<SwerveIO> {
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

  // Desired states for the swerve drive
  public enum WantedState {
    SYS_ID,
    FIELD_CENTRIC,
    ROBOT_CENTRIC,
    CHOREO_PATH,
    ROTATION_LOCK,
    TRACTOR_BEAM,
    IDLE
  }

  // Current system states for the swerve drive
  public enum SystemState {
    SYS_ID,
    FIELD_CENTRIC,
    ROBOT_CENTRIC,
    CHOREO_PATH,
    ROTATION_LOCK,
    TRACTOR_BEAM,
    IDLE
  }

  // State identifiers for the swerve drive
  private SystemState system_state_ = SystemState.FIELD_CENTRIC;
  private WantedState wanted_state_ = WantedState.FIELD_CENTRIC;

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

  // SysId Routine
  private final SysIdRoutine sysId;

  /**
   * Creates a new Swerve subsystem with the specified gyro and module IOs.
   *
   * @param gyroIO
   * @param flModuleIO
   * @param frModuleIO
   * @param blModuleIO
   * @param brModuleIO
   */
  public Swerve(SwerveIO io) {
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

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, (state) -> DogLog.log("SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, (Subsystem) this));
  }

  @Override
  public void updateLogic(double timestamp) {
    // Update the system state based on the wanted state
    system_state_ = handleStateTransition();

    // Update the request to apply based on the system state
    switch (system_state_) {
      case SYS_ID:
        break;
      case FIELD_CENTRIC:
        io.current_request =
            field_centric_request_.withTwist(calculateSpeedsBasedOnJoystickInputs());
        break;
      case ROBOT_CENTRIC:
        io.current_request =
            robot_centric_request_.withTwist(calculateSpeedsBasedOnJoystickInputs());
        break;
      case TRACTOR_BEAM:
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
                Math.abs(tractor_beam_controller_.calculate(linear_distance, 0))
                    + friction_constant,
                max_lin_vel_for_tractor_beam_);
        double x_component = velocity_output * direction_of_travel.getCos();
        double y_component = velocity_output * direction_of_travel.getSin();

        DogLog.log("Swerve/TractorBeam/X Velocity Component", x_component);
        DogLog.log("Swerve/TractorBeam/Y Velocity Component", y_component);
        DogLog.log("Swerve/TractorBeam/Velocity Output", velocity_output);
        DogLog.log("Swerve/TractorBeam/Linear Distance", linear_distance);
        DogLog.log("Swerve/TractorBeam/Direction of Travel", direction_of_travel);
        DogLog.log("Swerve/TractorBeam/Desired Point", desired_tractor_beam_pose_);

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
        break;
      case ROTATION_LOCK:
        io.current_request = rotation_lock_request_.withTargetHeading(desired_rotation_lock_rot_);
        break;
      case CHOREO_PATH:
        if (choreo_sample_to_apply_.isPresent()) {
          SwerveSample sample = choreo_sample_to_apply_.get();
          DogLog.log("Swerve/Choreo/Timer Value", choreo_timer_.get());
          DogLog.log("Swerve/Choreo/Traj Name", desired_choreo_traj_.name());
          DogLog.log("Swerve/Choreo/Total time", desired_choreo_traj_.getTotalTime());
          DogLog.log("Swerve/Choreo/sample/Desired Pose", sample.getPose());
          DogLog.log("Swerve/Choreo/sample/Desired Chassis Speeds", sample.getChassisSpeeds());
          DogLog.log("Swerve/Choreo/sample/Module Forces X", sample.moduleForcesX());
          DogLog.log("Swerve/Choreo/sample/Module Forces Y", sample.moduleForcesY());
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

    DogLog.log("Swerve/ModuleStates", getModuleStates());
    DogLog.log("Swerve/ChassisSpeeds", getChassisSpeeds());
    DogLog.log("Swerve/Rotation", getRotation());
    DogLog.log("Swerve/Pose", getPose());
    DogLog.log("Swerve/Raw Gyro", getGyroRotation());
  }

  @Override
  public void reset() {}

  // ------------------------------------------------
  // Subsystem State Update Methods
  // ------------------------------------------------

  /**
   * Sets the desired state for the Swerve subsystem.
   *
   * @param state the desired chassis state
   */
  public void setWantedState(WantedState state) {
    wanted_state_ = state;
  }

  /**
   * Gets the current system state of the Swerve subsystem.
   *
   * @return the current system state
   */
  private SystemState handleStateTransition() {
    return switch (wanted_state_) {
      case SYS_ID -> SystemState.SYS_ID;
      case FIELD_CENTRIC -> SystemState.FIELD_CENTRIC;
      case ROBOT_CENTRIC -> SystemState.ROBOT_CENTRIC;
      case CHOREO_PATH -> {
        if (system_state_ != SystemState.CHOREO_PATH) {
          choreo_timer_.restart();
          choreo_sample_to_apply_ =
              desired_choreo_traj_.sampleAt(
                  choreo_timer_.get(), SwerveConstants.FLIP_TRAJECTORY_ON_RED);
          yield SystemState.CHOREO_PATH;
        } else {
          choreo_sample_to_apply_ =
              desired_choreo_traj_.sampleAt(
                  choreo_timer_.get(), SwerveConstants.FLIP_TRAJECTORY_ON_RED);
          yield SystemState.CHOREO_PATH;
        }
      }
      case ROTATION_LOCK -> SystemState.ROTATION_LOCK;
      case TRACTOR_BEAM -> SystemState.TRACTOR_BEAM;
      default -> SystemState.IDLE;
    };
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
    wanted_state_ = WantedState.CHOREO_PATH;
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
    wanted_state_ = WantedState.TRACTOR_BEAM;
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
    wanted_state_ = WantedState.TRACTOR_BEAM;
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
    wanted_state_ = WantedState.TRACTOR_BEAM;
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
    wanted_state_ = WantedState.TRACTOR_BEAM;
  }

  /**
   * Updates the internal target for the robot to face and begins ROTATION_LOCK
   *
   * @param rotation
   */
  public void setDesiredRotationLock(Rotation2d rotation) {
    desired_rotation_lock_rot_ = rotation;
    wanted_state_ = WantedState.ROTATION_LOCK;
  }

  public Command toggleFieldCentric() {
    return Commands.runOnce(
        () -> {
          if (system_state_ == SystemState.FIELD_CENTRIC) {
            setWantedState(WantedState.ROBOT_CENTRIC);
          } else {
            setWantedState(WantedState.FIELD_CENTRIC);
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
    if (system_state_ != SystemState.CHOREO_PATH) {
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

  // ------------------------------------------------
  // Characterization Methods
  // ------------------------------------------------

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      swerve_io_.getModules()[i].runCharacterization(output);
    }
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.dynamic(direction));
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = swerve_io_.getModules()[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += swerve_io_.getModules()[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }
}
