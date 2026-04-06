package frc.robot.subsystems.localization;

import com.marswars.subsystem.MwConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class LocalizationConstants extends MwConstants {

    // =============================================================================
    // ENUMS AND STATE DEFINITIONS
    // =============================================================================

    public enum LocalizationStates {
        FULL
    }

    // =============================================================================
    // APRIL TAG CONFIGURATION
    // =============================================================================

    public final AprilTagFieldLayout APRIL_TAG_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // =============================================================================
    // ODOMETRY FILTER CONFIGURATION
    // =============================================================================
    public final Matrix<N3, N1> DEFAULT_ODOM_COVARIANCE = VecBuilder.fill(0.1, 0.1, 0.1);

    // =============================================================================
    // VISION FILTER CONFIGURATION
    // =============================================================================
    // Vision covariance matrices for different focus modes
    // Standard deviations: [x (meters), y (meters), theta (radians)]
    public final Matrix<N3, N1> DEFAULT_VISION_STD_DEV = VecBuilder.fill(0.9, 0.9, 0.9);

    // Maximum allowed rotation difference between vision measurement and current pose (radians)
    // Measurements with larger rotation differences will be discarded
    public final double MAX_ROTATION_DIFFERENCE = Units.degreesToRadians(360.0);

    // Maximum allowed yaw rate (rotation speed) for accepting vision measurements (radians per
    // second)
    public final double YAW_RATE_DISCARD = Units.degreesToRadians(720.0);

    // Minimum number of visible tags required to perform a vision update
    public final int MIN_TAG_COUNT_FOR_VISION_UPDATE = 2;

    // =============================================================================
    // CONSTRUCTOR
    // =============================================================================

    public LocalizationConstants() {
        // Some constants require dynamic initialization like through the JSON loader
    }
}
