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
import frc.mw_lib.swerve_lib.SwerveMeasurments.GyroMeasurement;
import frc.mw_lib.swerve_lib.SwerveMeasurments.ModuleMeasurement;
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

    private LocalizationIO localization_io_;
    private SwerveDrivePoseEstimator smooth_pose_estimator_;
    private SwerveDrivePoseEstimator field_pose_estimator_;

    public LocalizationSubsystem() {
        // (Default State, Constants Class)
        super(LocalizationStates.ACTIVE, new LocalizationConstants());

        localization_io_ = new LocalizationIO();
        SwerveDriveKinematics kinematics = SwerveSubsystem.getInstance().getKinematics();
        Rotation2d gyro_angle = SwerveSubsystem.getInstance().getGyroRotation();
        SwerveModulePosition[] module_positions = SwerveSubsystem.getInstance().getModulePositions();

        smooth_pose_estimator_ = new SwerveDrivePoseEstimator(kinematics, gyro_angle, module_positions, CONSTANTS.START_POSE);
        field_pose_estimator_ = new SwerveDrivePoseEstimator(kinematics, gyro_angle, module_positions, CONSTANTS.START_POSE);
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(localization_io_);
    }

    @Override
    public void reset() {
        system_state_ = LocalizationStates.ACTIVE;
    }

    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case ACTIVE:
                    for(int i = 0; i < localization_io_.module_measurements_.size(); i++){
                        ModuleMeasurement module_measurement = localization_io_.module_measurements_.get(i);
                        GyroMeasurement gyro_measurement = localization_io_.gyro_measurements_.get(i);

                        // Update Smooth Pose Estimator
                        smooth_pose_estimator_.updateWithTime(module_measurement.timestamp, gyro_measurement.gyro_yaw, module_measurement.module_positions);

                        // Update Field Post Estimator
                        field_pose_estimator_.updateWithTime(module_measurement.timestamp, gyro_measurement.gyro_yaw, module_measurement.module_positions);
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
