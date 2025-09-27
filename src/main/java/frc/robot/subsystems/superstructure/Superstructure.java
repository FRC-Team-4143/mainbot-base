package frc.robot.subsystems.superstructure;

import frc.mw_lib.subsystem.MWSubsystem;
import frc.robot.Constants;

public class Superstructure
    extends MWSubsystem<
        SuperstructureIO, frc.robot.subsystems.superstructure.Superstructure.SuperstructureState> {

  // Current system states for the superstructure
  public enum SuperstructureState {
    AT_TARGET,
    UNSAFE_MOVE,
    SAFE_MOVE,
    RESCUE,
  }

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

  // Current system states for the superstructure
  public enum SystemState {}

  Superstructure(SuperstructureIO io) {
    this.io = io;
  }

  @Override
  public void handleStateTransition(SuperstructureState wanted) {
    system_state_ =
        switch (wanted) {
          case AT_TARGET -> SuperstructureState.AT_TARGET;
          default -> SuperstructureState.UNSAFE_MOVE;
        };
  }

  @Override
  public void updateLogic(double timestamp) {}

  @Override
  public void reset() {}
}
