package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Fahrenheit;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;

public class IntakeIOReal extends IntakeIO {
  private final TalonFX pivot_motor_;
  private final TalonFX roller_motor_;
  private final TimeOfFlight tof_;
  private final PositionVoltage pivotRequest;

  public IntakeIOReal(IntakeConstants constants) {
    super(constants);

    pivot_motor_ = new TalonFX(CONSTANTS.PIVOT_ID);
    roller_motor_ = new TalonFX(CONSTANTS.ROLLER_ID);
    tof_ = new TimeOfFlight(CONSTANTS.TIME_OF_FLIGHT_ID);
    pivotRequest = new PositionVoltage(0);

    pivot_motor_.getConfigurator().apply(CONSTANTS.PIVOT_CONFIG);
  }

  @Override
  public void readInputs(double timestamp) {
    // Update pivot motor inputs
    pivot_current_position = Rotation2d.fromRotations(pivot_motor_.getPosition().getValueAsDouble());
    pivot_applied_voltage = pivot_motor_.getMotorVoltage().getValueAsDouble();
    pivot_current = pivot_motor_.getTorqueCurrent().getValueAsDouble();
    pivot_temp = pivot_motor_.getDeviceTemp().getValue().in(Fahrenheit);

    // Update roller motor inputs
    roller_applied_voltage = roller_motor_.getMotorVoltage().getValueAsDouble();
    roller_current = roller_motor_.getTorqueCurrent().getValueAsDouble();
    roller_temp = roller_motor_.getDeviceTemp().getValue().in(Fahrenheit);

    // Update ToF sensor input
    tof_dist = tof_.getRange();
  }

  @Override
  public void writeOutputs(double timestamp) {
    roller_motor_.set(roller_target_output);
    pivot_motor_.setControl(pivotRequest.withPosition(pivot_target_position.getRotations()));
  }

  public void updatePivotGains(Slot0Configs gains) {
    DataLogManager.log("Updating Pivot Gains: " + gains.toString());
    pivot_motor_.getConfigurator().apply(gains);
    Slot0Configs current_gains = new Slot0Configs();
    pivot_motor_.getConfigurator().refresh(current_gains);
    DataLogManager.log("Updated Pivot Gains: " + current_gains.toString());
  }
}
