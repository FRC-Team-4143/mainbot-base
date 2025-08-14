// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.pose_estimator.PoseEstimator;
import frc.robot.subsystems.swerve.ChassisRequest.ChassisRequestParameters;
import frc.robot.subsystems.swerve.ChassisRequest.XPositiveReference;
import frc.robot.subsystems.swerve.Module.DriveControlMode;
import frc.robot.subsystems.swerve.Module.SteerControlMode;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private static Swerve instance_ = null;

  public static Swerve getInstance() {
    if (instance_ == null) {
      if (Constants.IS_ROBOT_REAL) {
        instance_ =
            new Swerve(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXAnalog(SwerveConstants.FL_MODULE_CONSTANTS),
                new ModuleIOTalonFXAnalog(SwerveConstants.FR_MODULE_CONSTANTS),
                new ModuleIOTalonFXAnalog(SwerveConstants.BL_MODULE_CONSTANTS),
                new ModuleIOTalonFXAnalog(SwerveConstants.BR_MODULE_CONSTANTS));
      } else {
        instance_ =
            new Swerve(
                new GyroIOPigeon2(),
                new ModuleIOSim(SwerveConstants.FL_MODULE_CONSTANTS),
                new ModuleIOSim(SwerveConstants.FR_MODULE_CONSTANTS),
                new ModuleIOSim(SwerveConstants.BL_MODULE_CONSTANTS),
                new ModuleIOSim(SwerveConstants.BR_MODULE_CONSTANTS));
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
  @AutoLogOutput private SystemState system_state_ = SystemState.FIELD_CENTRIC;
  @AutoLogOutput private WantedState wanted_state_ = WantedState.FIELD_CENTRIC;

  // State specific variables
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

  static final Lock odometry_lock_ = new ReentrantLock();

  // Gyro Setup
  private final GyroIO gyro_IO_;
  private final GyroIOInputsAutoLogged gyro_inputs_ = new GyroIOInputsAutoLogged();
  private final Alert gyro_disconnected_alert_ =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  // Swerve Module Setup
  private final Module[] modules_ = new Module[4]; // FL, FR, BL, BR
  private SwerveDriveKinematics kinematics_ = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d raw_gyro_rotation_ = new Rotation2d();
  private SwerveModulePosition[] last_module_positions_ = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  // Chassis Request Parameters
  private ChassisRequestParameters request_parameters_ = new ChassisRequestParameters();

  private Rotation2d operator_forward_direction_ =
      SwerveConstants.OperatorPerspective.BLUE_ALLIANCE.heading;

  private ChassisRequest request_to_apply_ = new ChassisRequest.Idle();
  private ChassisRequest.FieldCentric field_centric_request_;
  private ChassisRequest.RobotCentric robot_centric_request_;
  private ChassisRequest.FieldCentricFacingAngle rotation_lock_request_;
  private ChassisRequest.ApplyFieldSpeeds field_speeds_request_;

  private final SysIdRoutine sysId;

  // private final SysIdRoutine sysId;

  /**
   * Creates a new Swerve subsystem with the specified gyro and module IOs.
   *
   * @param gyroIO
   * @param flModuleIO
   * @param frModuleIO
   * @param blModuleIO
   * @param brModuleIO
   */
  public Swerve(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyro_IO_ = gyroIO;
    modules_[0] = new Module(flModuleIO, 0, SwerveConstants.FL_MODULE_CONSTANTS);
    modules_[1] = new Module(frModuleIO, 1, SwerveConstants.FR_MODULE_CONSTANTS);
    modules_[2] = new Module(blModuleIO, 2, SwerveConstants.BL_MODULE_CONSTANTS);
    modules_[3] = new Module(brModuleIO, 3, SwerveConstants.BR_MODULE_CONSTANTS);

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
                null, null, null, (state) -> Logger.recordOutput("SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, (Subsystem) this));
  }

  @Override
  public void periodic() {
    odometry_lock_.lock(); // Prevents odometry updates while reading data
    gyro_IO_.updateInputs(gyro_inputs_);
    Logger.processInputs("Swerve/Gyro", gyro_inputs_);
    for (var module : modules_) {
      module.periodic();
    }
    odometry_lock_.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules_) {
        module.stop();
      }
    }

    // Update odometry
    double[] sampleTimestamps =
        modules_[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules_[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - last_module_positions_[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        last_module_positions_[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyro_inputs_.connected && Constants.IS_ROBOT_REAL) {
        // Use the real gyro angle
        raw_gyro_rotation_ = gyro_inputs_.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics_.toTwist2d(moduleDeltas);
        raw_gyro_rotation_ = raw_gyro_rotation_.plus(new Rotation2d(twist.dtheta));
      }
    }

    // Update the system state based on the wanted state
    system_state_ = handleStateTransition();

    // Update the request to apply based on the system state
    switch (system_state_) {
      case SYS_ID:
        break;
      case FIELD_CENTRIC:
        request_to_apply_ =
            field_centric_request_.withTwist(calculateSpeedsBasedOnJoystickInputs());
        break;
      case ROBOT_CENTRIC:
        request_to_apply_ =
            robot_centric_request_.withTwist(calculateSpeedsBasedOnJoystickInputs());
        break;
      case TRACTOR_BEAM:
        Translation2d translation_to_desired_point =
            desired_tractor_beam_pose_
                .getTranslation()
                .minus(PoseEstimator.getInstance().getSwerveOdometryPose().getTranslation());
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

        Logger.recordOutput("TractorBeam/X Velocity Component", x_component);
        Logger.recordOutput("TractorBeam/Y Velocity Component", y_component);
        Logger.recordOutput("TractorBeam/Velocity Output", velocity_output);
        Logger.recordOutput("TractorBeam/Linear Distance", linear_distance);
        Logger.recordOutput("TractorBeam/Direction of Travel", direction_of_travel);
        Logger.recordOutput("TractorBeam/Desired Point", desired_tractor_beam_pose_);

        if (Double.isNaN(max_ang_vel_for_tractor_beam_)) {
          request_to_apply_ =
              rotation_lock_request_
                  .withTwist(new Twist2d(x_component, y_component, 0.0))
                  .withTargetHeading(desired_tractor_beam_pose_.getRotation());
        } else {
          request_to_apply_ =
              rotation_lock_request_
                  .withTwist(new Twist2d(x_component, y_component, 0.0))
                  .withTargetHeading(desired_tractor_beam_pose_.getRotation())
                  .withMaxAbsRotationalRate(max_ang_vel_for_tractor_beam_);
        }
        break;
      case ROTATION_LOCK:
        request_to_apply_ = rotation_lock_request_.withTargetHeading(desired_rotation_lock_rot_);
        break;
      case CHOREO_PATH:
        if (choreo_sample_to_apply_.isPresent()) {
          SwerveSample sample = choreo_sample_to_apply_.get();
          Logger.recordOutput("Choreo/Timer Value", choreo_timer_.get());
          Logger.recordOutput("Choreo/Traj Name", desired_choreo_traj_.name());
          Logger.recordOutput("Choreo/Total time", desired_choreo_traj_.getTotalTime());
          Logger.recordOutput("Choreo/sample/Desired Pose", sample.getPose());
          Logger.recordOutput("Choreo/sample/Desired Chassis Speeds", sample.getChassisSpeeds());
          Logger.recordOutput("Choreo/sample/Module Forces X", sample.moduleForcesX());
          Logger.recordOutput("Choreo/sample/Module Forces Y", sample.moduleForcesY());
          Pose2d pose = PoseEstimator.getInstance().getSwerveOdometryPose();
          ChassisSpeeds target_speeds = sample.getChassisSpeeds();
          target_speeds.vxMetersPerSecond += choreo_x_controller_.calculate(pose.getX(), sample.x);
          target_speeds.vyMetersPerSecond += choreo_y_controller_.calculate(pose.getY(), sample.y);
          target_speeds.omegaRadiansPerSecond +=
              choreo_theta_controller_.calculate(pose.getRotation().getRadians(), sample.heading);

          request_to_apply_ = field_speeds_request_.withSpeeds(target_speeds);
        } else {
          // If no sample is available, we will just stop the robot
          request_to_apply_ = new ChassisRequest.Idle();
        }
        break;
      case IDLE:
      default:
        request_to_apply_ = new ChassisRequest.Idle();
        break;
    }

    // Set state static request parameters
    request_parameters_.kinematics = kinematics_;
    request_parameters_.currentChassisSpeed = getChassisSpeeds();
    request_parameters_.currentPose = PoseEstimator.getInstance().getSwerveOdometryPose();
    request_parameters_.updatePeriod = Timer.getFPGATimestamp() - request_parameters_.timestamp;
    request_parameters_.timestamp = Timer.getFPGATimestamp();
    request_parameters_.moduleLocations = getModuleTranslations();
    request_parameters_.operatorForwardDirection = operator_forward_direction_;

    request_to_apply_.apply(request_parameters_, modules_);
    Logger.recordOutput("swerveModuleStates", getModuleStates());
    Logger.recordOutput("chasisSpeed", getChassisSpeeds());
    Logger.recordOutput("gyroRotation", getGyroRotation());
    gyro_disconnected_alert_.set(!gyro_inputs_.connected && Constants.IS_ROBOT_REAL);
  }

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
    return runOnce(
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

    Twist2d Twist =
        new Twist2d(
            x_magnitude * SwerveConstants.MAX_TRANSLATION_RATE * tele_op_velocity_scalar_,
            y_magnitude * SwerveConstants.MAX_TRANSLATION_RATE * tele_op_velocity_scalar_,
            angular_magnitude * SwerveConstants.MAX_ANGULAR_RATE);
    Logger.recordOutput("joystickTwist", Twist);
    return Twist;
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
    var distance =
        desired_tractor_beam_pose_
            .getTranslation()
            .minus(PoseEstimator.getInstance().getSwerveOdometryPose().getTranslation())
            .getNorm();
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
            PoseEstimator.getInstance().getSwerveOdometryPose().getX(),
            SwerveConstants.CHOREO_TRANSLATION_ERROR_MARGIN)
        && MathUtil.isNear(
            desired_choreo_traj_.getFinalPose(true).get().getY(),
            PoseEstimator.getInstance().getSwerveOdometryPose().getY(),
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
                  PoseEstimator.getInstance().getSwerveOdometryPose().getX(),
                  SwerveConstants.CHOREO_TRANSLATION_ERROR_MARGIN))
              && MathUtil.isNear(
                  desired_choreo_traj_.getFinalPose(true).get().getY(),
                  PoseEstimator.getInstance().getSwerveOdometryPose().getY(),
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
                .minus(PoseEstimator.getInstance().getSwerveOdometryPose())
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
        desired_tractor_beam_pose_
            .getTranslation()
            .minus(PoseEstimator.getInstance().getSwerveOdometryPose().getTranslation())
            .getNorm();
    return diff;
  }

  // ------------------------------------------------
  // Chassis Property Methods
  // ------------------------------------------------

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules_[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules_[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics_.toChassisSpeeds(getModuleStates());
  }

  /** Returns the raw gyro rotation */
  public Rotation2d getGyroRotation() {
    return raw_gyro_rotation_;
  }

  /** Returns an array of module translations. */
  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      SwerveConstants.FL_MODULE_TRANSLATION,
      SwerveConstants.FR_MODULE_TRANSLATION,
      SwerveConstants.BL_MODULE_TRANSLATION,
      SwerveConstants.BR_MODULE_TRANSLATION
    };
  }

  /** Returns the kinematics object used to control the swerve frame */
  public SwerveDriveKinematics getKinematics() {
    return kinematics_;
  }

  // ------------------------------------------------
  // Characterization Methods
  // ------------------------------------------------

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules_[i].runCharacterization(output);
    }
  }

  // /** Returns a command to run a quasistatic test in the specified direction. */
  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return Commands.run(() -> runCharacterization(0.0))
  //       .withTimeout(1.0)
  //       .andThen(sysId.quasistatic(direction));
  // }

  // /** Returns a command to run a dynamic test in the specified direction. */
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return Commands.run(() -> runCharacterization(0.0))
  //       .withTimeout(1.0)
  //       .andThen(sysId.dynamic(direction));
  // }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules_[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules_[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }
}
