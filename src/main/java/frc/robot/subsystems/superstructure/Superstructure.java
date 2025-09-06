package frc.robot.subsystems.superstructure;

import frc.mw_lib.subsystem.MWSubsystem;
import frc.robot.Constants;

public class Superstructure extends MWSubsystem {

  private static Superstructure instance_ = null;

  public static Superstructure getInstance() {
    if (instance_ == null) {
      if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
        // Initialize with real IO components
        instance_ = new Superstructure(new SuperstructureIOReal());
      } else {
        instance_ = new Superstructure(new SuperstructureIOSim());
      }
    }
    return instance_;
  }

  // Desired states for the superstructure
  public enum WantedState {}

  // Current system states for the superstructure
  public enum SystemState {}


  Superstructure(SuperstructureIO io){
    this.io = io;
  }

  @Override
  public void updateLogic(double timestamp) {      
  }

  @Override
  public void reset() {
  }
}
