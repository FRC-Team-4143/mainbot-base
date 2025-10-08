package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Meters;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveConstants;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim extends IntakeIO {

  private final TalonFX pivot_motor_;
  private final TalonFX roller_motor_;
  private final DCMotor pivot_motor_sim_;
  private final DCMotor roller_motor_sim_;
  private final SingleJointedArmSim pivot_sim_;
  private final DCMotorSim roller_sim_;

  private final PositionVoltage pivot_control_request_ = new PositionVoltage(0);

  public IntakeIOSim(IntakeConstants constants) {
    super(constants);

    // Initialize the TalonFX motors
    pivot_motor_ = new TalonFX(CONSTANTS.PIVOT_ID);
    roller_motor_ = new TalonFX(CONSTANTS.ROLLER_ID);

    // Apply the configurations to the motors
    pivot_motor_.getConfigurator().apply(CONSTANTS.PIVOT_CONFIG);

    // Initialize the simulation objects
    IntakeConstants.INTAKE_SIMULATOR =
        IntakeSimulation.OverTheBumperIntake(
            "Coral",
            SwerveConstants.SWERVE_SIMULATOR,
            Meters.of(CONSTANTS.INTAKE_WIDTH),
            Meters.of(CONSTANTS.INTAKE_LENGTH_EXTENDED),
            IntakeSide.BACK,
            1);

    pivot_motor_sim_ = DCMotor.getKrakenX60(1);
    roller_motor_sim_ = DCMotor.getKrakenX60(1);

    pivot_sim_=
        new SingleJointedArmSim(
            pivot_motor_sim_,
            1.0 / CONSTANTS.PIVOT_MECH_RATIO,
            0.01,
            0.1,
            CONSTANTS.PIVOT_DEPLOYED_ANGLE.getRadians(),
            Units.degreesToRadians(70),
            false,
            70);

    roller_sim_ = new DCMotorSim(LinearSystemId.createDCMotorSystem(roller_motor_sim_, 0.01, 1.0), roller_motor_sim_);
  }

  @Override
  public void readInputs(double timestamp) {
    // Update pivot simulation
    pivot_motor_.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    pivot_sim_.setInput(pivot_motor_.getSimState().getMotorVoltage());
    pivot_sim_.update(0.02);

    // Update roller simulation
    roller_motor_.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    roller_sim_.setInput(roller_motor_.getSimState().getMotorVoltage());
    roller_sim_.update(0.02);

    // Simulate the battery voltage based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            pivot_sim_.getCurrentDrawAmps() + roller_sim_.getCurrentDrawAmps()));

    // Update simulation states with physics simulation results BEFORE next control cycle
    // This ensures the PID controller gets proper feedback from the physics simulation
    pivot_motor_
        .getSimState()
        .setRawRotorPosition(Units.radiansToRotations(pivot_sim_.getAngleRads() * CONSTANTS.PIVOT_MECH_RATIO));
    pivot_motor_
        .getSimState()
        .setRotorVelocity(Units.radiansToRotations(pivot_sim_.getVelocityRadPerSec() * CONSTANTS.PIVOT_MECH_RATIO));

    // Update roller motor simulation states
    roller_motor_
        .getSimState()
        .setRawRotorPosition(Units.radiansToRotations(roller_sim_.getAngularPositionRad()));
    roller_motor_
        .getSimState()
        .setRotorVelocity(Units.radiansToRotations(roller_sim_.getAngularVelocityRadPerSec()));

    // Update pivot motor inputs
    pivot_current_position = Rotation2d.fromRotations(pivot_motor_.getPosition().getValueAsDouble());
    pivot_applied_voltage = pivot_motor_.getMotorVoltage().getValueAsDouble();
    pivot_current = pivot_motor_.getTorqueCurrent().getValueAsDouble();
    pivot_temp = pivot_motor_.getDeviceTemp().getValue().in(Fahrenheit);

    // Update roller motor inputs
    roller_applied_voltage = roller_motor_.getMotorVoltage().getValueAsDouble();
    roller_current = roller_motor_.getTorqueCurrent().getValueAsDouble();
    roller_temp = roller_motor_.getDeviceTemp().getValue().in(Fahrenheit);

    // Update the tof sensor input
    tof_dist =
        (IntakeConstants.INTAKE_SIMULATOR.getGamePiecesAmount() > 0)
            ? CONSTANTS.TOF_CORAL_DISTANCE / 2
            : CONSTANTS.TOF_CORAL_DISTANCE * 2; // Simulated Time of Flight sensor distance
  }

  @Override
  public void writeOutputs(double timestamp) {
    roller_motor_.set(roller_target_output);
    pivot_motor_.setControl(pivot_control_request_.withPosition(pivot_target_position.getRotations()));

    // Start/stop the MapleSim intake based on roller output
    if (roller_target_output > 0) {
      IntakeConstants.INTAKE_SIMULATOR.startIntake();
    } else if (roller_target_output < 0) {
      IntakeConstants.INTAKE_SIMULATOR.obtainGamePieceFromIntake();
    } else {
      IntakeConstants.INTAKE_SIMULATOR.stopIntake();
    }
  }

  public void updatePivotGains(Slot0Configs gains) {
    DataLogManager.log("Updating Pivot Gains: " + gains.toString());
    pivot_motor_.getConfigurator().apply(gains);
    Slot0Configs current_gains = new Slot0Configs();
    pivot_motor_.getConfigurator().refresh(current_gains);
    DataLogManager.log("Updated Pivot Gains: " + current_gains.toString());
  }
}
