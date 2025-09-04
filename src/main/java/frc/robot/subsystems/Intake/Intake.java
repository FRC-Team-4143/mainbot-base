package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public enum WantedState {
    IDLE,
    PICKUP,
    PURGE,
    CLIMB_STAGE
  }

  public enum SystemState {
    DEPLOYING,
    PURGING,
    CLIMB_STAGING,
    PICKING_UP,
    IDLING
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;
  private final IntakeIO intake_io;
  private final IntakeIOInputsAutoLogged intake_io_inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO IO){
    intake_io = IO;
  }

  @Override
  public void periodic() {
    intake_io.updateInputs(intake_io_inputs);

    switch (systemState) {
        case DEPLOYING:
        
    }

  }
}
