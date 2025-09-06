package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeIOReal extends IntakeIO {
  private final TalonFX pivot_motor_;
  private final TalonFX roller_motor_;
  private final TimeOfFlight tof_;
  private final PositionVoltage pivotRequest;

  public IntakeIOReal() {
    pivot_motor_ = new TalonFX(0);
    roller_motor_ = new TalonFX(0);
    tof_ = new TimeOfFlight(0);
    pivotRequest = new PositionVoltage(0);
  }

  @Override
  public void readInputs(double timestamp) {
    current_pivot_angle = Rotation2d.fromRotations(pivot_motor_.getPosition().getValueAsDouble());
    tof_dist = tof_.getRange();
  }

  @Override
  public void writeOutputs(double timestamp) {
    roller_motor_.set(roller_output);
    pivot_motor_.setControl(pivotRequest.withPosition(target_pivot_angle.getRotations()));
  }
}
