package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.mw_lib.subsystem.MWSubsystem;
import frc.robot.Constants;

public class Intake extends MWSubsystem<IntakeIO, frc.robot.subsystems.intake.Intake.IntakeStates> {

  public enum IntakeStates {
    DEPLOY,
    PURGE,
    CLIMB_STAGE,
    PICK_UP,
    IDLE
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

  /**
   * Constructor for the Intake subsystem.
   * @param io the IntakeIO implementation to use for hardware interaction
   */
  public Intake(IntakeIO io) {
    super(IntakeStates.IDLE);
    this.io = io;

    this.system_state_ = IntakeStates.IDLE;
  }

  @Override
  public void updateLogic(double timestamp) {
    switch(system_state_){
      case DEPLOY:
        io.target_pivot_angle = Rotation2d.fromDegrees(0.0);
        io.roller_output = 0.5;
        break;
      case PURGE:
        io.target_pivot_angle = Rotation2d.fromDegrees(0.0);
        io.roller_output = 0.5;
        break;
      case CLIMB_STAGE:
        io.target_pivot_angle = Rotation2d.fromDegrees(0.0);
        io.roller_output = 0.5;
        break;
      case PICK_UP:
        io.target_pivot_angle = Rotation2d.fromDegrees(0.0);
        io.roller_output = 0.5;
        break;
      case IDLE:
      default:
        io.target_pivot_angle = Rotation2d.fromDegrees(0.0);
        io.roller_output = 0.5;
        break;
    }
  }

  @Override
  public void reset() {}
}
