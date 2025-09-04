package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeIOReal implements IntakeIO {
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

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.angle = Rotation2d.fromRotations(pivot_motor_.getPosition().getValueAsDouble());
    inputs.tofDist = tof_.getRange();
  }

  public void setRollerSpeed(double speed) {
    pivot_motor_.set(speed);
  }

  public void setIntakeAngle(Rotation2d angle) {
    roller_motor_.setControl(pivotRequest.withPosition(angle.getRotations()));
  }
}
