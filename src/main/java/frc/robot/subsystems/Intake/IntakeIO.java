package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.mw_lib.subsystem.SubsystemIO;

public abstract class IntakeIO extends SubsystemIO<IntakeConstants> {
  public IntakeIO(IntakeConstants constants) {
    super(constants);
  }

  public Rotation2d current_pivot_angle = Rotation2d.kZero;
  public double tof_dist = 0.0;

  public Rotation2d target_pivot_angle = current_pivot_angle;
  public double roller_output = 0.0;
}
