package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.mw_lib.subsystem.MWSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;

public class Intake extends MWSubsystem<IntakeIO, IntakeStates, IntakeConstants> {

  private static Intake instance_ = null;

  public static Intake getInstance() {
    if (instance_ == null) {
      instance_ = new Intake();
    }
    return instance_;
  }

  /**
   * Constructor for the Intake subsystem.
   */
  public Intake() {
    super(IntakeStates.IDLE, new IntakeConstants());

    // Figure out what I/O Container to use
    if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
      this.io = new IntakeIOReal(CONSTANTS);
    } else {
      this.io = new IntakeIOSim(CONSTANTS);
    }

    this.system_state_ = IntakeStates.IDLE;
  }

  @Override
  public void updateLogic(double timestamp) {
    switch (system_state_) {
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
  public void reset() {
  }
}
