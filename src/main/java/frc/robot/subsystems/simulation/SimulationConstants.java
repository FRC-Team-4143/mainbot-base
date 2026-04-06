package frc.robot.subsystems.simulation;

import com.marswars.subsystem.MwConstants;
import com.marswars.util.ConstantsLoader;

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

    // =============================================================================
    // CONSTRUCTOR
    // =============================================================================

    public SimulationConstants() {}
}
