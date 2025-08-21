package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.ChassisRequest.ChassisRequestParameters;
import frc.robot.subsystems.swerve.module.Module;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {
  @AutoLog
  public class SwerveIOInputs {
    public SwerveModuleState[] module_states_ =
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
        };
    public SwerveModulePosition[] module_positions_ =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };
    public SwerveModulePosition[] module_deltas_ =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };
    public SwerveModulePosition[] last_module_positions_ =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };
    public ChassisSpeeds chassis_speeds_ = new ChassisSpeeds();
    public Rotation2d raw_gyro_rotation_ = Rotation2d.kZero;
    public Pose2d pose_ = new Pose2d();
  }

  default void updateInputs(SwerveIOInputs inputs) {}

  default void applyRequest(ChassisRequest request, ChassisRequestParameters request_parameters) {}

  default void resetPose(Pose2d pose) {}

  default Module[] getModules() {
    return new Module[4];
  }
}
