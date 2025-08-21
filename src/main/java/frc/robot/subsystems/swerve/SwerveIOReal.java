package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.swerve.ChassisRequest.ChassisRequestParameters;
import frc.robot.subsystems.swerve.gyro.Gyro;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleIOTalonFXReal;

public class SwerveIOReal implements SwerveIO {

  private final Module[] modules_ = new Module[4]; // FL, FR, BL, BR
  private final Gyro gyro_;

  private final SwerveDrivePoseEstimator pose_estimator_;
  private final SwerveDriveKinematics kinematics_ =
      new SwerveDriveKinematics(getModuleTranslations());

  SwerveIOReal() {
    // Configure Gyro
    gyro_ = new Gyro(new GyroIOPigeon2());
    // Configure Modules
    modules_[0] =
        new Module(
            new ModuleIOTalonFXReal(SwerveConstants.FL_MODULE_CONSTANTS),
            0,
            SwerveConstants.FL_MODULE_CONSTANTS);
    modules_[1] =
        new Module(
            new ModuleIOTalonFXReal(SwerveConstants.FR_MODULE_CONSTANTS),
            1,
            SwerveConstants.FR_MODULE_CONSTANTS);
    modules_[2] =
        new Module(
            new ModuleIOTalonFXReal(SwerveConstants.BL_MODULE_CONSTANTS),
            2,
            SwerveConstants.BL_MODULE_CONSTANTS);
    modules_[3] =
        new Module(
            new ModuleIOTalonFXReal(SwerveConstants.BR_MODULE_CONSTANTS),
            3,
            SwerveConstants.BR_MODULE_CONSTANTS);

    // Configure Pose Estimator
    pose_estimator_ =
        new SwerveDrivePoseEstimator(
            kinematics_, new Rotation2d(), new SwerveModulePosition[4], Pose2d.kZero);
    // Setup MapleSim Drive Train Simulation
  }

  @Override
  public void updateInputs(SwerveIOInputs inputs) {
    Swerve.odometry_lock_.lock(); // Prevents odometry updates while reading data
    for (var module : modules_) {
      module.periodic();
    }
    Swerve.odometry_lock_.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules_) {
        module.stop();
      }
    }

    // Update odometry
    double[] sample_timestamps =
        modules_[0].getOdometryTimestamps(); // All signals are sampled together
    int sample_count = sample_timestamps.length;
    for (int i = 0; i < sample_count; i++) {
      // Read wheel positions and deltas from each module
      for (int module_index = 0; module_index < 4; module_index++) {
        inputs.module_positions_[module_index] = modules_[module_index].getOdometryPositions()[i];
        inputs.module_deltas_[module_index] =
            new SwerveModulePosition(
                inputs.module_positions_[module_index].distanceMeters
                    - inputs.last_module_positions_[module_index].distanceMeters,
                inputs.module_positions_[module_index].angle);
        inputs.last_module_positions_[module_index] = inputs.module_positions_[module_index];
        inputs.module_states_[module_index] = modules_[module_index].getState();
      }
      inputs.chassis_speeds_ = kinematics_.toChassisSpeeds(inputs.module_states_);

      // Update gyro angle
      if (gyro_.isConnected()) {
        // Use the real gyro angle
        inputs.raw_gyro_rotation_ = gyro_.getOdometryYawPositions()[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics_.toTwist2d(inputs.module_deltas_);
        inputs.raw_gyro_rotation_ = inputs.raw_gyro_rotation_.plus(new Rotation2d(twist.dtheta));
      }
      pose_estimator_.updateWithTime(
          sample_timestamps[i], inputs.raw_gyro_rotation_, inputs.module_positions_);
    }
    inputs.pose_ = pose_estimator_.getEstimatedPosition();
  }

  public void applyRequest(ChassisRequest request, ChassisRequestParameters request_parameters) {
    request_parameters.kinematics = kinematics_;
    request_parameters.moduleLocations = getModuleTranslations();
    request.apply(request_parameters, modules_);
  }

  public void resetPose(Pose2d pose) {
    pose_estimator_.resetPosition(pose.getRotation(), new SwerveModulePosition[4], pose);
  }

  public Module[] getModules() {
    return modules_;
  }

  private Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      modules_[0].getTranslation(), // FL
      modules_[1].getTranslation(), // FR
      modules_[2].getTranslation(), // BL
      modules_[3].getTranslation() // BR
    };
  }
}
