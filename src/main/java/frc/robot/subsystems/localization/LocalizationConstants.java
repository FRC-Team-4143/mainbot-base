package frc.robot.subsystems.localization;

import edu.wpi.first.math.geometry.Pose2d;
import frc.mw_lib.subsystem.MwConstants;

public class LocalizationConstants extends MwConstants {

    // Subsystem States
    public enum LocalizationStates {
        ACTIVE
    }

    public final Pose2d START_POSE = Pose2d.kZero;

    public LocalizationConstants() {
        // Some constants require dynamic initialization like through the JSON loader
    }
}
