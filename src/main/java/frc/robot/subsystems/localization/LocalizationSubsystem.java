package frc.robot.subsystems.localization;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.mw_lib.subsystem.MwSubsystem;
import frc.mw_lib.subsystem.SubsystemIoBase;
import frc.mw_lib.swerve_lib.PhoenixOdometryThread;
import frc.mw_lib.swerve_lib.SwerveMeasurments.GyroMeasurement;
import frc.mw_lib.swerve_lib.SwerveMeasurments.ModuleMeasurement;
import frc.mw_lib.swerve_lib.SwerveMeasurments.SwerveMeasurement;
import frc.robot.subsystems.localization.LocalizationConstants.LocalizationStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class LocalizationSubsystem extends MwSubsystem<LocalizationStates, LocalizationConstants> {
    private static LocalizationSubsystem instance_ = null;

    // Singleton Accessor
    public static LocalizationSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new LocalizationSubsystem();
        }
        return instance_;
    }

    private SwerveDrivePoseEstimator smooth_pose_estimator_;
    private SwerveDrivePoseEstimator field_pose_estimator_;
    private List<SwerveMeasurement> swerve_measurements_;


    public LocalizationSubsystem() {
        // (Default State, Constants Class)
        super(LocalizationStates.ACTIVE, new LocalizationConstants());

        SwerveDriveKinematics kinematics = SwerveSubsystem.getInstance().getKinematics();
        Rotation2d gyro_angle = SwerveSubsystem.getInstance().getGyroRotation();
        SwerveModulePosition[] module_positions = SwerveSubsystem.getInstance().getModulePositions();

        smooth_pose_estimator_ = new SwerveDrivePoseEstimator(kinematics, gyro_angle, module_positions, CONSTANTS.START_POSE);
        field_pose_estimator_ = new SwerveDrivePoseEstimator(kinematics, gyro_angle, module_positions, CONSTANTS.START_POSE);
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList();
    }

    @Override
    public void reset() {
        system_state_ = LocalizationStates.ACTIVE;
    }

    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case ACTIVE:
                swerve_measurements_ = PhoenixOdometryThread.getInstance().getSwerveSamples();
                    for(int i = 0; i < swerve_measurements_.size(); i++){
                        // Update Smooth Pose Estimator
                        smooth_pose_estimator_.updateWithTime(swerve_measurements_.get(i).timestamp, swerve_measurements_.get(i).gyro_yaw, swerve_measurements_.get(i).module_positions);

                        // Update Field Post Estimator
                        field_pose_estimator_.updateWithTime(swerve_measurements_.get(i).timestamp, swerve_measurements_.get(i).gyro_yaw, swerve_measurements_.get(i).module_positions);
                    }
                break;
        }
    }

    // Only override if you need custom state transitions!
    // @Override
    // public void handleStateTransition(LocalizationStates wanted) {
    // }

    /**
     * @return The smoothed pose estimate of the robot.
     */
    public Pose2d getSmoothPose(){
        return smooth_pose_estimator_.getEstimatedPosition();
    }

    /**
     * @return The field-relative pose estimate of the robot.
     */
    public Pose2d getFieldPose(){
        return field_pose_estimator_.getEstimatedPosition();
    }

    // Private Helper Methods
}
