package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.mw_lib.subsystem.SubsystemIO;
import frc.robot.subsystems.swerve.ChassisRequest.ChassisRequestParameters;
import frc.robot.subsystems.swerve.module.Module;

public class SwerveIO implements SubsystemIO {
  public SwerveModuleState[] module_states =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };
  public SwerveModulePosition[] module_positions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  public SwerveModulePosition[] module_deltas =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  public SwerveModulePosition[] last_module_positions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  public ChassisSpeeds chassis_speeds = new ChassisSpeeds();
  public Rotation2d raw_gyro_rotation = Rotation2d.kZero;
  public Pose2d pose = new Pose2d();

  public ChassisRequest current_request = new ChassisRequest.Idle();
  public ChassisRequestParameters current_request_parameters = new ChassisRequestParameters();

  void resetPose(Pose2d pose) {}

  Module[] getModules() {
    return new Module[4];
  }
}
