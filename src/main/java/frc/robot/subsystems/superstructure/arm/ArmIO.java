package frc.robot.subsystems.superstructure.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    public double position_ = 0.0; // Position in radians
    public double velocity_ = 0.0; // Velocity in radians per second
    public double applied_voltage_ = 0.0;
    public double current_ = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}
}
