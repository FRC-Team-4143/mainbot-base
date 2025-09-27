package frc.robot.subsystems.superstructure;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.mw_lib.subsystem.MWSubsystem;
import frc.robot.Constants;

public class Superstructure
    extends MWSubsystem<SuperstructureIO, frc.robot.subsystems.superstructure.Superstructure.SuperstructureState> {

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

  private List<SuperstructureTargets> targets_;

  Superstructure(SuperstructureIO io) {
    super(SuperstructureState.SAFE_MOVE);
    this.io = io;

    this.targets_ = new ArrayList<>();
  }

  @Override
  public void handleStateTransition(SuperstructureState wanted) {
    switch (wanted) {
      case AT_TARGET:
        if (targets_.size() > 0) {

        } else if (targets_.size() > 0) {

        } else {
          // No transititon
        }
        break;
      case UNSAFE_MOVE:

        break;
      case SAFE_MOVE:

        break;
      case RESCUE:

        break;
      default:
        DataLogManager.log("Unhandled state in Superstructure transition " + wanted.name());
    }
    ;
  }

  @Override
  public void updateLogic(double timestamp) {
    switch(system_state_){
      case AT_TARGET:
        
        break;
      case UNSAFE_MOVE:

        break;
      case SAFE_MOVE:

        break;
      case RESCUE:

        break;
      default:
        DataLogManager.log("Unhandled state in Superstructure logic " + system_state_.name());
    }
  }

  @Override
  public void reset() {
  }
}
