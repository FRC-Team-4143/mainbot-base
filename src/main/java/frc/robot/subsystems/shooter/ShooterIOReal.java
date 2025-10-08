package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Fahrenheit;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

public class ShooterIOReal extends ShooterIO {

  private final TalonFX roller_motor_;
  private final TimeOfFlight tof_;

  public ShooterIOReal(ShooterConstants constants) {
    super(constants);

    // Initialize the TalonFX motors
    roller_motor_ = new TalonFX(CONSTANTS.ROLLER_ID);
    tof_ = new TimeOfFlight(CONSTANTS.TIME_OF_FLIGHT_ID);

    // Apply the configurations to the motors
    roller_motor_.getConfigurator().apply(CONSTANTS.ROLLER_CONFIG);
  }

  @Override
  public void readInputs(double timestamp) {
    // Update roller motor inputs
    roller_applied_voltage = roller_motor_.getMotorVoltage().getValueAsDouble();
    roller_current = roller_motor_.getTorqueCurrent().getValueAsDouble();
    roller_temp = roller_motor_.getDeviceTemp().getValue().in(Fahrenheit);
    roller_current_velocity = roller_motor_.getVelocity().getValueAsDouble();

    // Update the tof sensor input
    tof_dist = tof_.getRange();
  }

  @Override
  public void writeOutputs(double timestamp) {
    roller_motor_.set(roller_target_output);
  }
}
