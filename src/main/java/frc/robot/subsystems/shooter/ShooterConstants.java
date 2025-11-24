package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import frc.mw_lib.subsystem.MwConstants;

public class ShooterConstants extends MwConstants {

    public enum ShooterStates {
        ACTIVE
    }

    public final double wheel_circumference_meters;
    public final double track_width_meters;

    public ShooterConstants() {
        wheel_circumference_meters = Units.inchesToMeters(6);
        track_width_meters = Units.inchesToMeters(24);
    }

}
