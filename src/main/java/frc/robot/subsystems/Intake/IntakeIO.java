package frc.robot.subsystems.Intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public Rotation2d angle = Rotation2d.kZero;
    public double tofDist = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
}
