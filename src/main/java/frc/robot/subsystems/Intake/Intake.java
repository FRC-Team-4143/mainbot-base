package frc.robot.subsystems.intake;

import frc.mw_lib.subsystem.MWSubsystem;
import frc.robot.Constants;

public class Intake extends MWSubsystem {

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

  public Intake(IntakeIO io){
    this.io = io;
  }

  @Override
  public void updateLogic(double timestamp) {
    
  }

  @Override
  public void reset() {
      
  }
}
