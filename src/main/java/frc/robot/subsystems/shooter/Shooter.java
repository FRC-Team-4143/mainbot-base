package frc.robot.subsystems.shooter;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.mw_lib.subsystem.MWSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;

public class Shooter extends MWSubsystem<ShooterIO, ShooterStates, ShooterConstants> {

  private static Shooter instance_ = null;

  public static Shooter getInstance() {
    if (instance_ == null) {
      instance_ = new Shooter();
    }
    return instance_;
  }

  // The target game piece for the shooter states
  private GamePiece target_game_piece_ = GamePiece.CORAL;

  // Debounce timers for game piece detection
  private final Debouncer coral_debouncer_ = new Debouncer(0.25, Debouncer.DebounceType.kBoth);
  private final Debouncer algae_debouncer_ = new Debouncer(0.25, Debouncer.DebounceType.kBoth);

  /** Constructor for the Intake subsystem. */
  public Shooter() {
    super(ShooterStates.IDLE, new ShooterConstants());

    // Figure out what I/O Container to use
    if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
      this.io = new ShooterIOReal(CONSTANTS);
    } else {
      this.io = new ShooterIOSim(CONSTANTS);
    }

    // initial state is assumed to be the deploy position
    this.system_state_ = ShooterStates.IDLE;
  }

  @Override
  public void handleStateTransition(ShooterStates wanted) {
    if(target_game_piece_ == GamePiece.ALGAE && wanted == ShooterStates.IDLE){
      wanted = ShooterStates.HOLDING_ALGAE;
    } else{
      system_state_ = wanted;
    }
  }

  @Override
  public void updateLogic(double timestamp) {
    switch (system_state_) {
      case SHOOTING_CORAL:
        io.roller_target_output = 1.0;
        break;
      case SHOOTING_ALGAE:
        io.roller_target_output = 0.5;
        break;
      case GRABBING_CORAL:
        io.roller_target_output = -1.0;
        break;
      case GRABBING_ALGAE:
        io.roller_target_output = -0.5;
        break;
      case HOLDING_ALGAE:
        io.roller_target_output = -0.1;
        break;
      case IDLE:
      default:
        io.roller_target_output = 0.0;
        break;
    }

    // Log the subsystem state
    DogLog.log(getSubsystemKey() + "State", system_state_);

    DogLog.log(getSubsystemKey() + "TargetGamePiece", target_game_piece_);
    DogLog.log(getSubsystemKey() + "HasCoral", hasCoral());
    DogLog.log(getSubsystemKey() + "HasAlgae", hasAlgae());
    SmartDashboard.putString("Target Game Piece", (target_game_piece_ == GamePiece.CORAL) ? Constants.CORAL_COLOR.toHexString() : Constants.ALGAE_COLOR.toHexString());
  }

  /** Returns true if the intake has a coral piece within the shooter */
  public boolean hasCoral(){
    return coral_debouncer_.calculate(io.tof_dist < CONSTANTS.TOF_CORAL_DISTANCE);
  }

  /** Returns true if the intake has a algae piece within the shooter */
  public boolean hasAlgae(){
    return algae_debouncer_.calculate(io.roller_current > 0 && io.roller_current_velocity < CONSTANTS.ALGAE_VELOCITY_THRESHOLD && target_game_piece_ == GamePiece.ALGAE);
  }

  /** Sets the target game piece for the shooter states */
  public void setTargetGamePiece(GamePiece game_piece){
    target_game_piece_ = game_piece;
  }

  /** Returns the current target game piece for the shooter states */
  public GamePiece getTargetGamePiece(){
    return target_game_piece_;
  }

  /** Toggles the target game piece for the shooter states */
  public Command toggleTargetGamePiece(){
    return Commands.runOnce(() -> {
      if(target_game_piece_ == GamePiece.CORAL){
        target_game_piece_ = GamePiece.ALGAE;
      } else {
        target_game_piece_ = GamePiece.CORAL;
      }
    });
  }

  /** Sets the shooter to shoot the current target game piece */
  public void shootGamePiece(){
    if(target_game_piece_ == GamePiece.CORAL){
      setWantedState(ShooterStates.SHOOTING_CORAL);
    } else {
      setWantedState(ShooterStates.SHOOTING_ALGAE);
    }
  }

  /** Sets the shooter to shoot the current target game piece */
  public void grabGamePiece(){
    if(target_game_piece_ == GamePiece.CORAL){
      setWantedState(ShooterStates.GRABBING_CORAL);
    } else {
      setWantedState(ShooterStates.GRABBING_CORAL);
    }
  }

  /** Sets the shooter to idle */
  public void stop(){
    setWantedState(ShooterStates.IDLE);
  }

  @Override
  public void reset() {}
}
