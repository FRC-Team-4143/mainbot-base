package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.mw_lib.subsystem.MWSubsystem;
import frc.mw_lib.util.NumUtil;
import frc.mw_lib.util.TunablePid;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.SuperstructureConstants.SuperstructureStates;
import java.util.ArrayList;
import java.util.List;

/**
 * Semantic Design for this system.
 *
 * <p>A Safe move is a move that has no interim target.
 *
 * <p>An unsafe move may have more than one target to reach its destination
 *
 * <p>Rescue is a special mode that is invoked if the elevator is above its IDLE position and the
 * arm is outside a given threshold. It will automatically take over when the system is re-enabled
 */
public class Superstructure
    extends MWSubsystem<SuperstructureIO, SuperstructureStates, SuperstructureConstants> {
  private static Superstructure instance_ = null;

  public static Superstructure getInstance() {
    if (instance_ == null) {
      // Initialize with real IO components
      instance_ = new Superstructure();
    }
    return instance_;
  }

  private List<SuperstructureTargets> targets_;

  Superstructure() {
    super(SuperstructureStates.AT_TARGET, new SuperstructureConstants());
    if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
      this.io = new SuperstructureIOReal(CONSTANTS);
    } else {
      this.io = new SuperstructureIOSim(CONSTANTS);
    }

    targets_ = new ArrayList<>();
    targets_.add(SuperstructureTargets.SAFETY);

    TunablePid.create(
        getSubsystemKey() + "/elevator",
        io::updateElevatorGains,
        CONSTANTS.ELEVATOR_LEADER_CONFIG.Slot0);
    TunablePid.create(
        getSubsystemKey() + "/arm", io::updateArmGains, CONSTANTS.ARM_MOTOR_CONFIG.Slot0);
  }

  @Override
  public void handleStateTransition(SuperstructureStates wanted) {
    // determine if we can update the number of targets. Only care if we are in a
    // moving state
    if (system_state_ == SuperstructureStates.SAFE_MOVE) {
      // if()
    } else if (system_state_ == SuperstructureStates.UNSAFE_MOVE) {

    }

    // if we have more than 1 target, we have been told to move
    // Two targets is an unsafe move and one target is a safe move
    if (targets_.size() > 2) {
      system_state_ = SuperstructureStates.UNSAFE_MOVE;
    } else if (targets_.size() > 1) {
      system_state_ = SuperstructureStates.SAFE_MOVE;
    } else {
      system_state_ = SuperstructureStates.AT_TARGET;
    }

    // We only allow an external request for Rescue. This is treated as forcible
    // override. We also bump the wanted state back to a safe move
    if (wanted == SuperstructureStates.RESCUE) {
      system_state_ = SuperstructureStates.RESCUE;
      setWantedState(SuperstructureStates.SAFE_MOVE);
    }
  }

  @Override
  public void updateLogic(double timestamp) {
    switch (system_state_) {
      case AT_TARGET:
        // intentional do nothing
        break;
      case UNSAFE_MOVE:
        // Run the current target orientation but the next target height
        // io.target_arm_position = ;
        // io.target_elevator_position = ;
        break;
      case SAFE_MOVE:
        // Run the current target orientation and target height
        break;
      case RESCUE:
        break;
      default:
        DataLogManager.log("Unhandled state in Superstructure logic " + system_state_.name());
    }
  }

  public boolean armAtTarget(SuperstructureTargets target) {
    return NumUtil.epislonEquals(
        target.getAngle().getRadians(), io.current_arm_position, CONSTANTS.ARM_TOLERANCE);
  }

  public boolean elevatorAtTarget(SuperstructureTargets target) {
    return NumUtil.epislonEquals(
        target.getHeight(), io.current_elevator_position, CONSTANTS.ELEV_TOLERANCE);
  }

  public boolean systemAtTarget(SuperstructureTargets target) {
    return armAtTarget(target) && elevatorAtTarget(target);
  }

  public ArrayList<SuperstructureTargets> isSafeMove(
      SuperstructureTargets begin, SuperstructureTargets dest) {
    // TODO (CJT) Implement this
    return new ArrayList<>();
  }

  public void requestMove(SuperstructureTargets target) {
    // first we need to check if this is a safe move

  }

  @Override
  public void reset() {}
}
