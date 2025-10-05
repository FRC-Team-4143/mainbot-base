package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.Slot0Configs;

public class SuperstructureIOSim extends SuperstructureIO {

  public SuperstructureIOSim(SuperstructureConstants constants) {
    super(constants);
  }

  /** Updates the set of loggable inputs. */
  public void readInputs(double timestamp) {}

  /** Writes the desired outputs to the motors. */
  public void writeOutputs(double timestamp) {}

  /** Zeroes the elevator position. */
  public void tarePosition() {}

  /**
   * Updates the gains for the elevator.
   *
   * @param gains The new gains to apply.
   */
  public void updateGains(Slot0Configs gains) {}
}
