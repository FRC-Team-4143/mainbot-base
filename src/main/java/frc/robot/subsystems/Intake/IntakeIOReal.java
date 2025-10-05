package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeIOReal extends IntakeIO {
  private final TalonFX pivot_motor_;
  private final TalonFX roller_motor_;
  private final TimeOfFlight tof_;
  private final PositionVoltage pivotRequest;

  public IntakeIOReal(IntakeConstants constants) {
    super(constants);
    
    pivot_motor_ = new TalonFX(IntakeConstants.PIVOT_ID);
    roller_motor_ = new TalonFX(IntakeConstants.INTAKE_ID);
    tof_ = new TimeOfFlight(IntakeConstants.TIME_OF_FLIGHT_ID);
    pivotRequest = new PositionVoltage(0);

    pivot_motor_.getConfigurator().apply(IntakeConstants.PICKUP_GAINS);
    pivot_motor_.getConfigurator().apply(new CurrentLimitsConfigs()
    .withStatorCurrentLimit(IntakeConstants.STATOR_CURRENT_LIMIT)
    .withStatorCurrentLimitEnable(true));
    pivot_motor_.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.SENSOR_TO_MECHANISM_RATIO));
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
