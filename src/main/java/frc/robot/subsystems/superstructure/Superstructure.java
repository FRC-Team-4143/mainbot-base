package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.arm.ArmIO;
import frc.robot.subsystems.superstructure.arm.ArmIOInputsAutoLogged;
import frc.robot.subsystems.superstructure.arm.ArmIOSim;
import frc.robot.subsystems.superstructure.arm.ArmIOTalonFX;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOInputsAutoLogged;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX;

public class Superstructure extends SubsystemBase {

  private static Superstructure instance_ = null;

  public static Superstructure getInstance() {
    if (instance_ == null) {
      if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
        // Initialize with real IO components
        instance_ =
            new Superstructure(
                new ElevatorIOTalonFX(ElevatorConstants.ELEVATOR_CONFIG), new ArmIOTalonFX());
      } else if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
        // Initialize with simulated IO components
        instance_ = new Superstructure(new ElevatorIOSim(), new ArmIOSim());
      } else {
        instance_ = new Superstructure(new ElevatorIO() {}, new ArmIO() {});
      }
    }
    return instance_;
  }

  // Desired states for the superstructure
  public enum WantedState {}

  // Current system states for the superstructure
  public enum SystemState {}

  private final ElevatorIO elevator_io_;
  private final ArmIO arm_io_;

  private final ElevatorIOInputsAutoLogged elevator_io_inputs_auto_logged_ =
      new ElevatorIOInputsAutoLogged();
  private final ArmIOInputsAutoLogged arm_io_inputs_auto_logged_ = new ArmIOInputsAutoLogged();

  Superstructure(ElevatorIO elevatorIO, ArmIO armIO) {
    this.elevator_io_ = elevatorIO;
    this.arm_io_ = armIO;
  }
}
