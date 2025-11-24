package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.util.Units;
import frc.mw_lib.subsystem.MwConstants;

public class DrivetrainConstants extends MwConstants {

    public enum DrivetrainStates {
        IDLE,
        TELE_OP,
        AUTO,
    }

    public final double wheel_circumference_meters;
    public final double track_width_meters;

    public DrivetrainConstants() {
        wheel_circumference_meters = Units.inchesToMeters(6);
        track_width_meters = Units.inchesToMeters(24);
    }

}
