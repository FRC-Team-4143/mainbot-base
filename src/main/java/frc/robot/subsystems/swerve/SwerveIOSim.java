package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import dev.doglog.DogLog;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.mw_lib.swerve_lib.gyro.Gyro;
import frc.mw_lib.swerve_lib.gyro.GyroIOSim;
import frc.mw_lib.swerve_lib.module.Module;
import frc.mw_lib.swerve_lib.module.ModuleIOTalonFXSim;
import frc.robot.Constants;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class SwerveIOSim extends SwerveIO {

  private final Pose2d SIM_START_POSE = new Pose2d(3, 3, Rotation2d.kZero);

  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      SIM_MODULE_CONSTANTS_FACTORY =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(SwerveConstants.FL_MODULE_CONSTANTS.DriveMotorGearRatio)
              .withSteerMotorGearRatio(16.0) // Maximum gear ratio sim can support
              .withCouplingGearRatio(SwerveConstants.COUPLE_RATIO)
              .withWheelRadius(SwerveConstants.WHEEL_RADIUS_METERS)
              .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
              .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
              .withSlipCurrent(SwerveConstants.SLIP_CURRENT_AMPS)
              .withSpeedAt12Volts(SwerveConstants.SPEED_AT_12V_MPS)
              .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
              .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
              .withDriveMotorInitialConfigs(new TalonFXConfiguration())
              .withSteerMotorInitialConfigs(new TalonFXConfiguration())
              .withEncoderInitialConfigs(new CANcoderConfiguration())
              .withSteerInertia(KilogramSquareMeters.of(0.05)) // Adjust steer inertia
              .withDriveInertia(SwerveConstants.DRIVE_INERTIA)
              .withDriveFrictionVoltage(Volts.of(0.1)) // Adjust friction voltages
              .withSteerFrictionVoltage(Volts.of(0.05)) // Adjust friction voltages
              .withDriveMotorGains(SwerveConstants.FL_MODULE_CONSTANTS.DriveMotorGains)
              .withSteerMotorGains( // Adjust steer motor PID gains for simulation
                  new Slot0Configs()
                      .withKP(70)
                      .withKI(0)
                      .withKD(4.5)
                      .withKS(0)
                      .withKV(1.91)
                      .withKA(0)
                      .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));

  private static final SwerveModuleConstants SIM_FL_MODULE_CONSTANTS =
      SIM_MODULE_CONSTANTS_FACTORY.createModuleConstants(
          2,
          1,
          0,
          0,
          SwerveConstants.FL_MODULE_CONSTANTS.LocationX,
          SwerveConstants.FL_MODULE_CONSTANTS.LocationY,
          false,
          false,
          false);
  private static final SwerveModuleConstants SIM_FR_MODULE_CONSTANTS =
      SIM_MODULE_CONSTANTS_FACTORY.createModuleConstants(
          4,
          3,
          1,
          0,
          SwerveConstants.FR_MODULE_CONSTANTS.LocationX,
          SwerveConstants.FR_MODULE_CONSTANTS.LocationY,
          false,
          false,
          false);
  private static final SwerveModuleConstants SIM_BL_MODULE_CONSTANTS =
      SIM_MODULE_CONSTANTS_FACTORY.createModuleConstants(
          6,
          5,
          2,
          0,
          SwerveConstants.BL_MODULE_CONSTANTS.LocationX,
          SwerveConstants.BL_MODULE_CONSTANTS.LocationY,
          false,
          false,
          false);
  private static final SwerveModuleConstants SIM_BR_MODULE_CONSTANTS =
      SIM_MODULE_CONSTANTS_FACTORY.createModuleConstants(
          8,
          7,
          3,
          0,
          SwerveConstants.BR_MODULE_CONSTANTS.LocationX,
          SwerveConstants.BR_MODULE_CONSTANTS.LocationY,
          false,
          false,
          false);

  private final Module[] modules_ = new Module[4]; // FL, FR, BL, BR
  private final Gyro gyro_;

  private final SwerveDrivePoseEstimator pose_estimator_;
  private final SwerveDriveKinematics kinematics_ =
      new SwerveDriveKinematics(
          new Translation2d[] {
            SwerveConstants.FL_MODULE_TRANSLATION,
            SwerveConstants.FR_MODULE_TRANSLATION,
            SwerveConstants.BL_MODULE_TRANSLATION,
            SwerveConstants.BR_MODULE_TRANSLATION,
          });

  @SuppressWarnings("unchecked")
  private final DriveTrainSimulationConfig maple_sim_config_ =
      new DriveTrainSimulationConfig(
          Kilograms.of(SwerveConstants.ROBOT_MASS_KG),
          Meters.of(SwerveConstants.BUMPER_LENGTH_METERS),
          Meters.of(SwerveConstants.BUMPER_WIDTH_METERS),
          Meters.of(SIM_FL_MODULE_CONSTANTS.LocationX - SIM_BL_MODULE_CONSTANTS.LocationX),
          Meters.of(SIM_FL_MODULE_CONSTANTS.LocationY - SIM_FR_MODULE_CONSTANTS.LocationY),
          COTS.ofPigeon2(),
          new SwerveModuleSimulationConfig(
              DCMotor.getKrakenX60(1),
              DCMotor.getFalcon500(1),
              SIM_FL_MODULE_CONSTANTS.DriveMotorGearRatio,
              SIM_FL_MODULE_CONSTANTS.SteerMotorGearRatio,
              Volts.of(SIM_FL_MODULE_CONSTANTS.DriveFrictionVoltage),
              Volts.of(SIM_FL_MODULE_CONSTANTS.SteerFrictionVoltage),
              Meters.of(SIM_FL_MODULE_CONSTANTS.WheelRadius),
              KilogramSquareMeters.of(SIM_FL_MODULE_CONSTANTS.SteerInertia),
              SwerveConstants.WHEEL_COF));

  SwerveIOSim(SwerveConstants constants) {
    super(constants);
    // Configure MapleSim
    Constants.SWERVE_SIMULATOR = new SwerveDriveSimulation(maple_sim_config_, SIM_START_POSE);
    // Configure Gyro
    gyro_ = new Gyro(new GyroIOSim(Constants.SWERVE_SIMULATOR.getGyroSimulation()));
    // Configure Modules
    modules_[0] =
        new Module(
            new ModuleIOTalonFXSim(SIM_FL_MODULE_CONSTANTS, Constants.SWERVE_SIMULATOR.getModules()[0]),
            0,
            SIM_FL_MODULE_CONSTANTS);
    modules_[1] =
        new Module(
            new ModuleIOTalonFXSim(SIM_FR_MODULE_CONSTANTS, Constants.SWERVE_SIMULATOR.getModules()[1]),
            1,
            SIM_FR_MODULE_CONSTANTS);
    modules_[2] =
        new Module(
            new ModuleIOTalonFXSim(SIM_BL_MODULE_CONSTANTS, Constants.SWERVE_SIMULATOR.getModules()[2]),
            2,
            SIM_BL_MODULE_CONSTANTS);
    modules_[3] =
        new Module(
            new ModuleIOTalonFXSim(SIM_BR_MODULE_CONSTANTS, Constants.SWERVE_SIMULATOR.getModules()[3]),
            3,
            SIM_BR_MODULE_CONSTANTS);
    // Configure Pose Estimator
    pose_estimator_ =
        new SwerveDrivePoseEstimator(
            kinematics_,
            new Rotation2d(),
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            },
            SIM_START_POSE);
    // Setup MapleSim Drive Train Simulation
    SimulatedArena.getInstance().addDriveTrainSimulation(Constants.SWERVE_SIMULATOR);
  }

  @Override
  public void readInputs(double timestamp) {
    Swerve.odometry_lock_.lock(); // Prevents odometry updates while reading data
    for (var module : modules_) {
      module.periodic();
    }
    gyro_.periodic();
    Swerve.odometry_lock_.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules_) {
        module.stop();
      }
    }

    // Update odometry
    double[] sample_timestamps =
        modules_[0].getOdometryTimestamps(); // All signals are sampled together
    int sample_count = sample_timestamps.length;
    for (int i = 0; i < sample_count; i++) {
      // Read wheel positions and deltas from each module
      for (int module_index = 0; module_index < 4; module_index++) {
        module_positions[module_index] = modules_[module_index].getOdometryPositions()[i];
        module_deltas[module_index] =
            new SwerveModulePosition(
                module_positions[module_index].distanceMeters
                    - last_module_positions[module_index].distanceMeters,
                module_positions[module_index].angle);
        last_module_positions[module_index] = module_positions[module_index];
        module_states[module_index] = modules_[module_index].getState();
      }
      chassis_speeds = kinematics_.toChassisSpeeds(module_states);

      // Update gyro angle
      if (gyro_.isConnected()) {
        // Use the real gyro angle
        raw_gyro_rotation = gyro_.getOdometryYawPositions()[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics_.toTwist2d(module_deltas);
        raw_gyro_rotation = raw_gyro_rotation.plus(new Rotation2d(twist.dtheta));
      }
      pose_estimator_.updateWithTime(sample_timestamps[i], raw_gyro_rotation, module_positions);
    }
    pose = pose_estimator_.getEstimatedPosition();

    // Update Simulated Position
    DogLog.log("FieldSimulation/RobotPosition", Constants.SWERVE_SIMULATOR.getSimulatedDriveTrainPose());
  }

  public void writeOutputs(double timestamp) {
    current_request_parameters.kinematics = kinematics_;
    current_request_parameters.moduleLocations = getModuleTranslations();
    current_request.apply(current_request_parameters, modules_);
  }

  public void resetPose(Pose2d pose) {
    Constants.SWERVE_SIMULATOR.setSimulationWorldPose(pose);
    pose_estimator_.resetPosition(pose.getRotation(), new SwerveModulePosition[4], pose);
  }

  public Module[] getModules() {
    return modules_;
  }

  private Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      modules_[0].getTranslation(), // FL
      modules_[1].getTranslation(), // FR
      modules_[2].getTranslation(), // BL
      modules_[3].getTranslation() // BR
    };
  }
}
