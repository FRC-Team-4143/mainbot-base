package frc.robot.subsystems.pose_estimator;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.Swerve;

public class PoseEstimator extends SubsystemBase {
  private static PoseEstimator instance_;

  public static PoseEstimator getInstance() {
    if (instance_ == null) {
      instance_ = new PoseEstimator();
    }
    return instance_;
  }

  private SwerveDrivePoseEstimator pose_estimator_;
  private StructPublisher<Pose2d> robot_pose_pub_;

  PoseEstimator() {
    pose_estimator_ =
        new SwerveDrivePoseEstimator(
            Swerve.getInstance().getKinematics(),
            Swerve.getInstance().getGyroRotation(),
            Swerve.getInstance().getModulePositions(),
            new Pose2d());

    robot_pose_pub_ =
        NetworkTableInstance.getDefault()
            .getStructTopic("PoseEstimation/Robot Pose", Pose2d.struct)
            .publish();
  }

  @Override
  public void periodic() {
    pose_estimator_.update(
        Swerve.getInstance().getGyroRotation(), Swerve.getInstance().getModulePositions());
    robot_pose_pub_.set(getSwerveOdometryPose());
  }

  public Pose2d getSwerveOdometryPose() {
    return pose_estimator_.getEstimatedPosition();
  }
}
