package frc.robot.subsystems.simulation;

import com.marswars.proxy_server.ProxyServerThread;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import com.marswars.swerve_lib.SwerveMeasurements.SwerveMeasurement;
import com.marswars.vision.MwVisionSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.simulation.SimulationConstants.SimulationStates;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

public class SimulationSubsystem extends MwSubsystem<SimulationStates, SimulationConstants> {
    private static SimulationSubsystem instance_ = null;

    private MwVisionSim vision_sim_;
    private final Random noise_generator_ = new Random();

    // getInstance
    public static SimulationSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new SimulationSubsystem();
        }
        return instance_;
    }

    // Constructor
    public SimulationSubsystem() {
        super(SimulationStates.ACTIVE, new SimulationConstants());

        if (CONSTANTS.SIM_VISION_ENABLED) {
            vision_sim_ =
                    ProxyServerThread.getInstance()
                            .initializeVisionSimulation(
                                    LocalizationSubsystem.getInstance().getAprilTagLayout());
            vision_sim_.addCamera("Back-Camera", CONSTANTS.BACK_CAMERA_TRANSFORM);
            vision_sim_.addCamera("Left-Camera", CONSTANTS.LEFT_CAMERA_TRANSFORM);
            vision_sim_.addCamera("Right-Camera", CONSTANTS.RIGHT_CAMERA_TRANSFORM);
            LocalizationSubsystem.getInstance().enableSwerveMeasurementNoise();
        }

        // Setup autonomous reset
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::resetForAuto));
        resetForAuto();
    }

    // reset
    @Override
    public void reset() {
        system_state_ = SimulationStates.ACTIVE;
    }

    // getIos
    @Override
    public List<SubsystemIoBase> getIos() {
        // This subsystem has no IOs
        return Arrays.asList();
    }

    // updateLogic
    @Override
    public void updateLogic(double timestamp) {
        // Vision Simulation
        if (CONSTANTS.SIM_VISION_ENABLED) {
            Pose2d robot_pose = LocalizationSubsystem.getInstance().getSmoothPose();
            ProxyServerThread.getInstance().updateVisionSimulation(robot_pose);
        }
    }

    // =============================================================================
    // PUBLIC HELPER METHODS
    // =============================================================================

    /** Resets the simulation for autonomous mode. */
    public void resetForAuto() {
        // Move robot to starting pose
        LocalizationSubsystem.getInstance().resetPoseEstimatorAuto();
    }

    /**
     * Adds Gaussian noise to a complete swerve measurement. Applies noise to both gyro and module
     * position measurements.
     *
     * @param clean_measurement The clean swerve measurement
     * @return Swerve measurement with added noise
     */
    public SwerveMeasurement addNoise(SwerveMeasurement clean_measurement) {
        SwerveMeasurement noisy_measurement = clean_measurement;
        noisy_measurement.gyro_yaw = addGyroNoise(clean_measurement.gyro_yaw);
        noisy_measurement.module_positions =
                addModulePositionNoise(clean_measurement.module_positions);

        return noisy_measurement;
    }

    // =============================================================================
    // PRIVATE HELPER METHODS
    // =============================================================================

    /**
     * Adds Gaussian noise to gyro measurement for simulation.
     *
     * @param clean_gyro The clean gyro measurement
     * @return Gyro measurement with added noise
     */
    private Rotation2d addGyroNoise(Rotation2d clean_gyro) {
        double noise = noise_generator_.nextGaussian() * CONSTANTS.GYRO_NOISE_STD_DEV;
        return clean_gyro.plus(Rotation2d.fromRadians(noise));
    }

    /**
     * Adds Gaussian noise to module positions for simulation.
     *
     * @param clean_positions The clean module position measurements
     * @return Module positions with added noise
     */
    private SwerveModulePosition[] addModulePositionNoise(SwerveModulePosition[] clean_positions) {
        SwerveModulePosition[] noisy_positions = new SwerveModulePosition[clean_positions.length];
        for (int i = 0; i < clean_positions.length; i++) {
            double distance_noise =
                    noise_generator_.nextGaussian() * CONSTANTS.MODULE_POSITION_NOISE_STD_DEV;
            noisy_positions[i] =
                    new SwerveModulePosition(
                            clean_positions[i].distanceMeters + distance_noise,
                            clean_positions[i].angle);
        }
        return noisy_positions;
    }
}
