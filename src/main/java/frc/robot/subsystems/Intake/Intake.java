package frc.robot.subsystems.intake;

import frc.mw_lib.subsystem.MWSubsystem;
import frc.robot.Constants;

public class Intake extends MWSubsystem<IntakeIO, frc.robot.subsystems.intake.Intake.IntakeState> {

  public enum IntakeState {
    DEPLOYING,
    PURGING,
    CLIMB_STAGING,
    PICKING_UP,
    IDLING
  }

  private static Intake instance_ = null;

  public static Intake getInstance() {
    if (instance_ == null) {
      if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
        instance_ = new Intake(new IntakeIOReal());
      } else {
        instance_ = new Intake(new IntakeIOSim());
      }
    }
    return instance_;
  }

  public Intake(IntakeIO io) {
    super(IntakeState.IDLING);
    this.io = io;

    this.system_state_ = IntakeState.IDLING;
  }

  @Override
  public void updateLogic(double timestamp) {}

  @Override
  public void reset() {}
}
