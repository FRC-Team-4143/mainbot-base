package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.mw_lib.subsystem.MWSubsystem;
import frc.mw_lib.util.NumUtil;
import frc.mw_lib.util.TunablePid;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.SuperstructureConstants.SuperstructureStates;
import java.util.ArrayList;
import java.util.List;
import dev.doglog.DogLog;

/**
 * Semantic Design for this system.
 *
 * <p>
 * A Safe move is a move that has no interim target.
 *
 * <p>
 * An unsafe move may have more than one target to reach its destination
 *
 * <p>
 * Rescue is a special mode that is invoked if the elevator is above its IDLE
 * position and the
 * arm is outside a given threshold. It will automatically take over when the
 * system is re-enabled
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

  private List<SuperstructureTarget> targets_;

  Superstructure() {
    super(SuperstructureStates.AT_TARGET, new SuperstructureConstants());
    if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
      this.io = new SuperstructureIOReal(CONSTANTS);
    } else {
      this.io = new SuperstructureIOSim(CONSTANTS);
    }

    targets_ = new ArrayList<>();
    targets_.add(SuperstructureTarget.Targets.INTAKE_CLEAR.target);

    TunablePid.create(
        getSubsystemKey() + "/elevator",
        io::updateElevatorGains,
        CONSTANTS.ELEVATOR_LEADER_CONFIG.Slot0);
    TunablePid.create(
        getSubsystemKey() + "/arm", io::updateArmGains, CONSTANTS.ARM_MOTOR_CONFIG.Slot0);
  }

  @Override
  public void handleStateTransition(SuperstructureStates wanted) {
    // We only allow an external request for Rescue. This is treated as forcible
    // override. We also bump the wanted state back to a safe move
    if (wanted == SuperstructureStates.RESCUE) {
      setWantedState(SuperstructureStates.MOVING);

      // If we were moving outside the RESCUE height and arm angle, then we need to
      // rescue. Practically this means we do it for L4 only really
      if (io.current_elevator_position > CONSTANTS.ELEV_RESCUE_HEIGHT
          && io.current_arm_position > CONSTANTS.ARM_RESCUE_ANGLE) {
        double elevator_ht = NumUtil.clamp(io.current_elevator_position + 0.20, CONSTANTS.ELEVATOR_MIN_HEIGHT,
            CONSTANTS.ELEVATOR_MAX_HEIGHT);
        double arm_angle = io.current_arm_position;
        SuperstructureTarget rescue_target = new SuperstructureTarget("RESCUE", elevator_ht,
            Rotation2d.fromRadians(arm_angle));

        // inject a rescue target
        targets_.add(0, rescue_target);
      }
    }

    // determine if we can update the number of targets. Only care if we are in a
    // moving state. Also make sure we are looking at the correct height target
    if (system_state_ == SuperstructureStates.MOVING) {
      SuperstructureTarget next_tgt = targets_.get(1);

      // if the elevator and arm are at their next targets
      if (elevatorAtTargetInternal(next_tgt)
          && armAtTargetInternal(next_tgt)) {
        targets_.remove(0);
      }
    }

    // if we have more than 1 target, we have been told to move
    // Two targets is an unsafe move and one target is a safe move
    if (targets_.size() > 1) {
      system_state_ = SuperstructureStates.MOVING;
    } else {
      system_state_ = SuperstructureStates.AT_TARGET;
    }
  }

  @Override
  public void updateLogic(double timestamp) {
    switch (system_state_) {
      case AT_TARGET:
        // intentional do nothing
        break;
      case MOVING:
        // run the current arm position
        io.target_arm_position = targets_.get(1).getAngle().getRadians();
        io.target_elevator_position = targets_.get(1).getHeight();
        break;
      case RESCUE:
        break;
      default:
        DataLogManager.log("Unhandled state in Superstructure logic " + system_state_.name());
    }
  }

  protected boolean armAtTargetInternal(SuperstructureTarget target) {
    return NumUtil.epislonEquals(
        target.getAngle().getRadians(), io.current_arm_position, CONSTANTS.ARM_TOLERANCE);
  }

  protected boolean elevatorAtTargetInternal(SuperstructureTarget target) {
    return NumUtil.epislonEquals(
        target.getHeight(), io.current_elevator_position, CONSTANTS.ELEV_TOLERANCE);
  }

  public boolean systemAtTarget() {
    // If the subsystem state is at target, then definitely yes
    if (system_state_ == SuperstructureStates.AT_TARGET) {
      return true;
    }

    // Otherwise check if the elevator and arm are at their respective targets
    int final_tgt_idx = targets_.size() - 1;
    return armAtTargetInternal(targets_.get(final_tgt_idx))
        && elevatorAtTargetInternal(targets_.get(final_tgt_idx));
  }

  protected List<SuperstructureTarget> getSafeMove(
      SuperstructureTarget begin, SuperstructureTarget.Targets dest) {
    List<SuperstructureTarget> safe_targets = new ArrayList<>();

    // Anything from L4 needs staging
    if (begin.equals(SuperstructureTarget.Targets.L4.target)) {
      safe_targets.add(SuperstructureTarget.Targets.L4_STAGING.target);
    }

    // Anything to L4 need staging
    if (dest == SuperstructureTarget.Targets.L4) {
      safe_targets.add(SuperstructureTarget.Targets.L4_STAGING.target);
    }

    // anything to L1 has a special motion profile
    if (begin.equals(SuperstructureTarget.Targets.CORAL_INTAKE.target) && dest == SuperstructureTarget.Targets.L1) {
      safe_targets.add(SuperstructureTarget.Targets.L1_STAGING.target);
    }

    // always add the final target
    safe_targets.add(dest.target);

    return safe_targets;
  }

  public void requestMove(SuperstructureTarget.Targets target) {
    // figure out if the final target is the same as the requested move
    int final_tgt_idx = targets_.size() - 1;
    SuperstructureTarget final_target = targets_.get(final_tgt_idx);
    if (target.target != final_target) {
      // use the last target in the list (should always have an elevator height and
      // arm angle)

      List<SuperstructureTarget> new_targets = getSafeMove(final_target, target);

      targets_.addAll(new_targets);
    }
  }

  @Override
  public void reset() {
  }

  @Override
  public void logData() {
    // Log the subsystem state
    DogLog.log(getSubsystemKey() + "State", system_state_);

    // Log the current elevator target
    int next_target_idx = targets_.size() > 0 ? 1 : 0;
    DogLog.log(getSubsystemKey() + "Target/Name", targets_.get(next_target_idx).getName());

    // Log the upcoming targets in the queue
    String[] upcoming_targets = new String[targets_.size()];
    for(int i = 0; i < targets_.size(); i++){
      upcoming_targets[i] = targets_.get(i).getName();
    }
    DogLog.log(getSubsystemKey()+"Target/Upcoming", upcoming_targets);
  }
}
