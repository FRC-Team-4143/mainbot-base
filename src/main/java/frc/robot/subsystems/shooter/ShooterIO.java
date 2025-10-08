package frc.robot.subsystems.shooter;

import dev.doglog.DogLog;
import frc.mw_lib.subsystem.SubsystemIO;

public abstract class ShooterIO extends SubsystemIO<ShooterConstants> {
    public ShooterIO(ShooterConstants constants) {
        super(constants);
      }

  /** Current velocity of the roller in rot/s */
  public double roller_current_velocity = 0.0;

  /** Applied voltage to the roller */
  public double roller_applied_voltage = 0.0;

  /** Current in amperes for the roller */
  public double roller_current = 0.0;

  /** Temperature in degrees Fahrenheit for the roller */
  public double roller_temp = 0.0;

  /** Target output for the roller */
  public double roller_target_output = 0.0;

  /** Time-of-flight sensor distance */
  public double tof_dist = 0.0;

  /** Logs data to DogLog.  */
  @Override
  public void logData() {
    DogLog.log(getSubsystemKey() + "Roller/Velocity", roller_current_velocity);
    DogLog.log(getSubsystemKey() + "Roller/AppliedVoltage", roller_applied_voltage);
    DogLog.log(getSubsystemKey() + "Roller/Current", roller_current);
    DogLog.log(getSubsystemKey() + "Roller/Temp", roller_temp);
    DogLog.log(getSubsystemKey() + "Roller/TargetOutput", roller_target_output);

    DogLog.log(getSubsystemKey() + "TOFDistance", tof_dist);
  }
    
}
