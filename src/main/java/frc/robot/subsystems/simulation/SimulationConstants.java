package frc.robot.subsystems.simulation;

import com.marswars.subsystem.MwConstants;
import com.marswars.util.ConstantsLoader;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class SimulationConstants extends MwConstants {

    private final ConstantsLoader LOADER = ConstantsLoader.getInstance();

    // =============================================================================
    // ENUMS AND STATE DEFINITIONS
    // =============================================================================

    public enum SimulationStates {
        ACTIVE
    }

    // =============================================================================
    // POSE ESTIMATION SIMULATION
    // =============================================================================
    public final boolean SIM_VISION_ENABLED = false;
    public final double GYRO_NOISE_STD_DEV = Math.toRadians(0.5); // radians (0.5 degrees)
    public final double MODULE_POSITION_NOISE_STD_DEV = 0.01; // meters (1cm per reading)
    public final Transform3d BACK_CAMERA_TRANSFORM =
            new Transform3d(
                    -0.330,
                    0.288,
                    0.540,
                    new Rotation3d(
                            0.0, Units.degreesToRadians(-30.0), Units.degreesToRadians(180.0)));
    public final Transform3d RIGHT_CAMERA_TRANSFORM =
            new Transform3d(
                    -0.031,
                    -0.335,
                    0.202,
                    new Rotation3d(
                            0.0, Units.degreesToRadians(-35.0), Units.degreesToRadians(-90.0)));
    public final Transform3d LEFT_CAMERA_TRANSFORM =
            new Transform3d(
                    -0.239,
                    0.343,
                    0.540,
                    new Rotation3d(
                            0.0, Units.degreesToRadians(-10.0), Units.degreesToRadians(60.0)));

    // =============================================================================
    // CONSTRUCTOR
    // =============================================================================

    public SimulationConstants() {}
}
