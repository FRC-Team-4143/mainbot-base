package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.mw_lib.util.ConstantsLoader;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureTarget;
import frc.robot.subsystems.swerve.Swerve;

public class ShooterIOSim extends ShooterIO {

  private final TalonFX roller_motor_;
  private final DCMotor roller_motor_sim_;
  private final DCMotorSim roller_sim_;
  
  private final Debouncer coral_debouncer_ = new Debouncer(0.1, Debouncer.DebounceType.kRising);
  private final Debouncer algae_debouncer_ = new Debouncer(0.1, Debouncer.DebounceType.kRising);

  private final Translation2d SHOOTER_OFFSET = new Translation2d(Units.inchesToMeters(ConstantsLoader.getInstance().getDoubleValue("swerve", "com", "bumper_length")) / 2.0, 0.05);

  public ShooterIOSim(ShooterConstants constants) {
    super(constants);

    // Initialize the TalonFX motors
    roller_motor_ = new TalonFX(CONSTANTS.ROLLER_ID);

    // Apply the configurations to the motors
    roller_motor_.getConfigurator().apply(CONSTANTS.ROLLER_CONFIG);

    roller_motor_sim_ = DCMotor.getKrakenX60(1);
    roller_sim_ = new DCMotorSim(LinearSystemId.createDCMotorSystem(roller_motor_sim_, 0.01, 1.0), roller_motor_sim_);

    // Ensure the robot does not start with coral in the shooter
    tof_dist = CONSTANTS.TOF_CORAL_DISTANCE * 2;
  }

  @Override
  public void readInputs(double timestamp) {
    // Update roller simulation
    roller_motor_.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    roller_sim_.setInput(roller_motor_.getSimState().getMotorVoltage());
    roller_sim_.update(0.02);

    // Simulate the battery voltage based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            roller_sim_.getCurrentDrawAmps()));

    roller_motor_
        .getSimState()
        .setRawRotorPosition(Units.radiansToRotations(roller_sim_.getAngularPositionRad()));
    roller_motor_
        .getSimState()
        .setRotorVelocity(Units.radiansToRotations(roller_sim_.getAngularVelocityRadPerSec()));

    // Update roller motor inputs
    roller_applied_voltage = roller_motor_.getMotorVoltage().getValueAsDouble();
    roller_current = roller_motor_.getTorqueCurrent().getValueAsDouble();
    roller_temp = roller_motor_.getDeviceTemp().getValue().in(Fahrenheit);

    // Update the tof sensor input
    if(Intake.getInstance().hasCoral() // Intake
        && Shooter.getInstance().getTargetGamePiece() == GamePiece.CORAL // Shooter is in Coral Mode
        && Shooter.getInstance().hasCoral() == false // Shooter does not already have Coral
        && coral_debouncer_.calculate(roller_target_output < 0.0) // Roller is grabbing
        && Superstructure.getInstance().isSystemAtTarget(SuperstructureTarget.Targets.CORAL_INTAKE)){ // Superstructure is at Coral Intake Position
      tof_dist = CONSTANTS.TOF_CORAL_DISTANCE / 2.0;
    }

    // Update velocity for Algae detection
    if(Shooter.getInstance().getTargetGamePiece() == GamePiece.ALGAE
        && algae_debouncer_.calculate(roller_target_output < 0.0)){ // Roller is grabbing 
      roller_current_velocity = CONSTANTS.ALGAE_VELOCITY_THRESHOLD * 0.8; // Set velocity to 80% of algae threshold to simulate algae detection
    } else {
      roller_current_velocity = roller_motor_.getVelocity().getValueAsDouble();
    }
  }

  @Override
  public void writeOutputs(double timestamp) {
    roller_motor_.set(roller_target_output);

    if(roller_target_output > 0.0){
      if(Shooter.getInstance().hasCoral() && MathUtil.isNear(tof_dist, CONSTANTS.TOF_CORAL_DISTANCE / 2.0, 1E-6)) shootCoral(); 
      if(Shooter.getInstance().hasAlgae()) shootAlgae();  
    }
  }

  /** Simulate Coral being shoot from the shooter */
  private void shootCoral(){
    // Spawn a coral projectile in the arena
    SimulatedArena.getInstance().addGamePieceProjectile(
        new ReefscapeCoralOnFly(
          Swerve.getInstance().getPose().getTranslation(), 
          SHOOTER_OFFSET, 
          Swerve.getInstance().getChassisSpeeds(), 
          Swerve.getInstance().getPose().getRotation(), 
          Meters.of(Superstructure.getInstance().getCurrentEndEffectorPosition()), 
          MetersPerSecond.of(2), 
          Radians.of(Superstructure.getInstance().getCurrentEndEffectorAngle())));

    // Reset the tof distance to simulate the coral being shot out
    tof_dist = CONSTANTS.TOF_CORAL_DISTANCE * 2;
    IntakeConstants.INTAKE_SIMULATOR.obtainGamePieceFromIntake();
  }

  /** Simulate Algae being shoot from the shooter */
  private void shootAlgae(){
    SimulatedArena.getInstance().addGamePieceProjectile(
        new ReefscapeAlgaeOnFly(
          Swerve.getInstance().getPose().getTranslation(), 
          SHOOTER_OFFSET, 
          Swerve.getInstance().getChassisSpeeds(), 
          Swerve.getInstance().getPose().getRotation(), 
          Meters.of(Superstructure.getInstance().getCurrentEndEffectorPosition()), 
          MetersPerSecond.of(0.5), 
          Radians.of(Superstructure.getInstance().getCurrentEndEffectorAngle())));
  }

}
