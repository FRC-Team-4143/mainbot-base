package frc.robot.subsystems.intake;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.mw_lib.subsystem.MWSubsystem;
import frc.mw_lib.util.TunablePid;
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

  // Telemetry objects
  StructPublisher<Pose3d> current_pivot_pub_;

  // Tunable setpoints are tracked locally to avoid DogLog two-arg tunable NPE
  double tunable_target_pivot_position_;

  /** Constructor for the Intake subsystem. */
  public Intake() {
    super(IntakeStates.IDLE, new IntakeConstants());

    // Figure out what I/O Container to use
    if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
      this.io = new IntakeIOReal(CONSTANTS);
    } else {
      this.io = new IntakeIOSim(CONSTANTS);
    }

    // initial state is assumed to be the deploy position
    this.system_state_ = IntakeStates.DEPLOY;

    // Create tunable objects for the pivot and arm
    TunablePid.create(
        getSubsystemKey() + "Pivot",
        io::updatePivotGains,
        CONSTANTS.PIVOT_GAINS);

    // Use tunables with callbacks to update local setpoints. Avoid the two-arg overload
    // which can trigger an internal NPE in DogLog's Notifier thread.
    tunable_target_pivot_position_  = io.pivot_current_position.getRadians();
    DogLog.tunable(
    getSubsystemKey() + "Pivot/Setpoint",
    tunable_target_pivot_position_,
    newVal -> tunable_target_pivot_position_ = newVal);

    // Setup the pose3d publisher
    current_pivot_pub_ =
        NetworkTableInstance.getDefault()
            .getStructTopic(getSubsystemKey() + "Component", Pose3d.struct)
            .publish();
  }

  @Override
  public void updateLogic(double timestamp) {
    switch (system_state_) {
      case DEPLOY:
        io.pivot_target_position = CONSTANTS.PIVOT_DEPLOYED_ANGLE;
        setWantedState(IntakeStates.IDLE);
        break;
      case PURGE:
        io.roller_target_output = CONSTANTS.INTAKE_OUT_SPEED;
        break;
      case CLIMB_STAGE:
        io.pivot_target_position = CONSTANTS.PIVOT_CLIMB_ANGLE;
        io.roller_target_output = 0.0;
        setWantedState(IntakeStates.IDLE);
        break;
      case PICK_UP:
        io.pivot_target_position = CONSTANTS.PIVOT_DEPLOYED_ANGLE;
        io.roller_target_output = CONSTANTS.INTAKE_IN_SPEED;
        break;
      case TUNING:
        io.pivot_target_position = Rotation2d.fromDegrees(tunable_target_pivot_position_);
        io.roller_target_output = 0.0;
        break;
      case IDLE:
      default:
        io.roller_target_output = 0.0;
        break;
    }

    // Log the subsystem state
    DogLog.log(getSubsystemKey() + "State", system_state_);
    DogLog.log(getSubsystemKey() + "HasCoral", hasCoral());

    publishMechanisms();
  }

  /** Returns true if the intake has a coral piece within the intake mechanism */
  public boolean hasCoral(){
    return io.tof_dist < CONSTANTS.TOF_CORAL_DISTANCE;
  }

  /** Publish the mechanism 2d and pose3d objects to SmartDashboard */
  private void publishMechanisms(){
    // Update the pose3d publishers
    current_pivot_pub_.set(
        new Pose3d(
            -Units.inchesToMeters(11),
            0,
            Units.inchesToMeters(8.25),
            new Rotation3d(0, io.pivot_current_position.getRadians(), 0)));
  }

  @Override
  public void reset() {}
}
