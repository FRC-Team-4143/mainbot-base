package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.mw_lib.util.ConstantsLoader;
import frc.robot.Constants;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class SwerveConstants {

  // ConstantsLoader instance for loading configuration values
  private static final ConstantsLoader LOADER = ConstantsLoader.getInstance();

  // CAN bus name and CAN ID for the pigeon2
  public static final int PIGEON2_ID = 0;
  public static final String PIGEON2_CANBUS_NAME = "CANivore";

  // CAN bus names for each of the swerve modules
  public static final String MODULE_CANBUS_NAME = "CANivore";

  // Determine the CAN bus update frequency for odometry
  public static final double ODOMETRY_FREQUENCY =
      new CANBus(MODULE_CANBUS_NAME).isNetworkFD() ? 250.0 : 100.0;

  public static final double DRIVE_INERTIA = 0.025; // kg*m^2, inertia of the drive motor
  public static final double STEER_INERTIA = 0.004; // kg*m^2, inertia of the steer motor
  public static final double DRIVE_FRICTION_VOLTAGE = 0.2;
  public static final double STEER_FRICTION_VOLTAGE = 0.2;

  public static final double ROBOT_MASS_LBS = 140.0; // Mass of the robot in pounds
  public static final double WHEEL_COF = 1.2; // Coefficient of friction for the wheels

  // Forward Reference rotation constants
  public enum OperatorPerspective {
    BLUE_ALLIANCE(Rotation2d.fromDegrees(0.0)),
    RED_ALLIANCE(Rotation2d.fromDegrees(180.0));

    private OperatorPerspective(Rotation2d heading) {
      this.heading = heading;
    }

    public final Rotation2d heading;
  }

  // The steer motor uses MotionMagicVoltage control
  private static final Slot0Configs STEER_GAINS =
      new Slot0Configs()
          .withKP(LOADER.getDoubleValue("drive", "com", "STEER_GAINS_P"))
          .withKI(LOADER.getDoubleValue("drive", "com", "STEER_GAINS_I"))
          .withKD(LOADER.getDoubleValue("drive", "com", "STEER_GAINS_D"))
          .withKS(LOADER.getDoubleValue("drive", "com", "STEER_GAINS_S"))
          .withKV(LOADER.getDoubleValue("drive", "com", "STEER_GAINS_V"))
          .withKA(LOADER.getDoubleValue("drive", "com", "STEER_GAINS_A"));
  // When using closed-loop control, the drive motor uses VelocityVoltage
  private static final Slot0Configs DRIVE_GAINS =
      new Slot0Configs()
          .withKP(LOADER.getDoubleValue("drive", "com", "DRIVE_GAINS_P"))
          .withKI(LOADER.getDoubleValue("drive", "com", "DRIVE_GAINS_I"))
          .withKD(LOADER.getDoubleValue("drive", "com", "DRIVE_GAINS_D"))
          .withKS(LOADER.getDoubleValue("drive", "com", "DRIVE_GAINS_S"))
          .withKV(LOADER.getDoubleValue("drive", "com", "DRIVE_GAINS_V"))
          .withKA(LOADER.getDoubleValue("drive", "com", "DRIVE_GAINS_A"));

  // Control Constants for the swerve modules
  private static final double SLIP_CURRENT_AMPS =
      LOADER.getDoubleValue("drive", "com", "SLIP_CURRENT");
  public static final double SPEED_AT_12V_MPS =
      LOADER.getDoubleValue("drive", "com", "SPEED_AT_12V");
  private static final double COUPLE_RATIO = LOADER.getDoubleValue("drive", "com", "COUPLE_RATIO");
  private static final double WHEEL_RADIUS_METERS =
      Units.inchesToMeters(LOADER.getDoubleValue("drive", "com", "WHEEL_RADIUS_INCH"));
  public static final double MAX_TRANSLATION_RATE =
      LOADER.getDoubleValue("drive", "com", "MAX_DRIVE_SPEED");
  public static final double MAX_ANGULAR_RATE =
      LOADER.getDoubleValue("drive", "com", "MAX_DRIVE_ANGULAR_RATE");

  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      ConstantCreator =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              // .withDriveMotorGearRatio(kDriveGearRatio) Unique to Module Type
              // .withSteerMotorGearRatio(kSteerGearRatio) Unique to Module Type
              .withCouplingGearRatio(COUPLE_RATIO)
              .withWheelRadius(WHEEL_RADIUS_METERS)
              // .withSteerMotorGains(steerGains) Unique to Module Type
              // .withDriveMotorGains(driveGains) Unique to Module Type
              .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
              .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
              .withSlipCurrent(SLIP_CURRENT_AMPS)
              .withSpeedAt12Volts(SPEED_AT_12V_MPS)
              .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
              .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
              // .withFeedbackSource(kSteerFeedbackType) Not set for Analog Encoder
              .withDriveMotorInitialConfigs(new TalonFXConfiguration())
              .withSteerMotorInitialConfigs(new TalonFXConfiguration())
              .withEncoderInitialConfigs(new CANcoderConfiguration())
              .withSteerInertia(STEER_INERTIA)
              .withDriveInertia(DRIVE_INERTIA)
              .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
              .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

  // Front Left Module Constants
  private static final ModuleType FL_MODULE_TYPE = ModuleType.getModuleType("fl");
  public static final Translation2d FL_MODULE_TRANSLATION =
      new Translation2d(
          Units.inchesToMeters(LOADER.getDoubleValue("drive", "fl", "X_POSITION")),
          Units.inchesToMeters(LOADER.getDoubleValue("drive", "fl", "Y_POSITION")));
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FL_MODULE_CONSTANTS =
          ConstantCreator.createModuleConstants(
                  LOADER.getIntValue("drive", "fl", "STEER_ID"),
                  LOADER.getIntValue("drive", "fl", "DRIVE_ID"),
                  LOADER.getIntValue("drive", "fl", "ENCODER_ID"),
                  0,
                  FL_MODULE_TRANSLATION.getX(),
                  FL_MODULE_TRANSLATION.getY(),
                  LOADER.getBoolValue("drive", "fl", "INVERT_DRIVE"),
                  FL_MODULE_TYPE.steerInverted,
                  false)
              .withDriveMotorGearRatio(FL_MODULE_TYPE.driveRatio)
              .withSteerMotorGearRatio(FL_MODULE_TYPE.steerRatio);

  // Front Right Module Constants
  private static final ModuleType FR_MODULE_TYPE = ModuleType.getModuleType("fr");
  public static final Translation2d FR_MODULE_TRANSLATION =
      new Translation2d(
          Units.inchesToMeters(LOADER.getDoubleValue("drive", "fr", "X_POSITION")),
          Units.inchesToMeters(LOADER.getDoubleValue("drive", "fr", "Y_POSITION")));
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FR_MODULE_CONSTANTS =
          ConstantCreator.createModuleConstants(
                  LOADER.getIntValue("drive", "fr", "STEER_ID"),
                  LOADER.getIntValue("drive", "fr", "DRIVE_ID"),
                  LOADER.getIntValue("drive", "fr", "ENCODER_ID"),
                  0,
                  FR_MODULE_TRANSLATION.getX(),
                  FR_MODULE_TRANSLATION.getY(),
                  LOADER.getBoolValue("drive", "fr", "INVERT_DRIVE"),
                  FR_MODULE_TYPE.steerInverted,
                  false)
              .withDriveMotorGearRatio(
                  (Constants.CURRENT_MODE != Constants.Mode.SIM)
                      ? FR_MODULE_TYPE.driveRatio
                      : FL_MODULE_TYPE.driveRatio) // Use FL drive ratio in sim
              .withSteerMotorGearRatio(
                  (Constants.CURRENT_MODE != Constants.Mode.SIM)
                      ? FR_MODULE_TYPE.steerRatio
                      : FL_MODULE_TYPE.steerRatio); // Use FL steer ratio in sim

  // Back Left Module Constants
  private static final ModuleType BL_MODULE_TYPE = ModuleType.getModuleType("bl");
  public static final Translation2d BL_MODULE_TRANSLATION =
      new Translation2d(
          Units.inchesToMeters(LOADER.getDoubleValue("drive", "bl", "X_POSITION")),
          Units.inchesToMeters(LOADER.getDoubleValue("drive", "bl", "Y_POSITION")));
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BL_MODULE_CONSTANTS =
          ConstantCreator.createModuleConstants(
                  LOADER.getIntValue("drive", "bl", "STEER_ID"),
                  LOADER.getIntValue("drive", "bl", "DRIVE_ID"),
                  LOADER.getIntValue("drive", "bl", "ENCODER_ID"),
                  0,
                  BL_MODULE_TRANSLATION.getX(),
                  BL_MODULE_TRANSLATION.getY(),
                  LOADER.getBoolValue("drive", "bl", "INVERT_DRIVE"),
                  BL_MODULE_TYPE.steerInverted,
                  false)
              .withDriveMotorGearRatio(
                  (Constants.CURRENT_MODE != Constants.Mode.SIM)
                      ? BL_MODULE_TYPE.driveRatio
                      : FL_MODULE_TYPE.driveRatio) // Use FL drive ratio in sim
              .withSteerMotorGearRatio(
                  (Constants.CURRENT_MODE != Constants.Mode.SIM)
                      ? BL_MODULE_TYPE.steerRatio
                      : FL_MODULE_TYPE.steerRatio); // Use FL steer ratio in sim

  // Back Right Module Constants
  private static final ModuleType BR_MODULE_TYPE = ModuleType.getModuleType("br");
  public static final Translation2d BR_MODULE_TRANSLATION =
      new Translation2d(
          Units.inchesToMeters(LOADER.getDoubleValue("drive", "br", "X_POSITION")),
          Units.inchesToMeters(LOADER.getDoubleValue("drive", "br", "Y_POSITION")));
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BR_MODULE_CONSTANTS =
          ConstantCreator.createModuleConstants(
                  LOADER.getIntValue("drive", "br", "STEER_ID"),
                  LOADER.getIntValue("drive", "br", "DRIVE_ID"),
                  LOADER.getIntValue("drive", "br", "ENCODER_ID"),
                  0,
                  BR_MODULE_TRANSLATION.getX(),
                  BR_MODULE_TRANSLATION.getY(),
                  LOADER.getBoolValue("drive", "br", "INVERT_DRIVE"),
                  BR_MODULE_TYPE.steerInverted,
                  false)
              .withDriveMotorGearRatio(
                  (Constants.CURRENT_MODE != Constants.Mode.SIM)
                      ? BR_MODULE_TYPE.driveRatio
                      : FL_MODULE_TYPE.driveRatio) // Use FL drive ratio in sim
              .withSteerMotorGearRatio(
                  (Constants.CURRENT_MODE != Constants.Mode.SIM)
                      ? BR_MODULE_TYPE.steerRatio
                      : FL_MODULE_TYPE.steerRatio); // Use FL steer ratio in sim

  // Choreo Constants
  public static final boolean FLIP_TRAJECTORY_ON_RED = true;
  public static final double CHOREO_TRANSLATION_ERROR_MARGIN = Units.inchesToMeters(1.0);
  public static final double CHOREO_X_CONTROLLER_KP = 0.0;
  public static final double CHOREO_X_CONTROLLER_KI = 0.0;
  public static final double CHOREO_X_CONTROLLER_KD = 0.0;
  public static final double CHOREO_Y_CONTROLLER_KP = 0.0;
  public static final double CHOREO_Y_CONTROLLER_KI = 0.0;
  public static final double CHOREO_Y_CONTROLLER_KD = 0.0;
  public static final double CHOREO_THETA_CONTROLLER_KP = 0.0;
  public static final double CHOREO_THETA_CONTROLLER_KI = 0.0;
  public static final double CHOREO_THETA_CONTROLLER_KD = 0.0;

  // Tractor Beam Constants
  public static final double TRACTOR_BEAM_TRANSLATION_ERROR_MARGIN = Units.inchesToMeters(0.5);
  public static final double TRACTOR_BEAM_STATIC_FRICTION_CONSTANT = 0.1;
  public static final double TRACTOR_BEAM_CONTROLLER_KP = 0.0;
  public static final double TRACTOR_BEAM_CONTROLLER_KI = 0.0;
  public static final double TRACTOR_BEAM_CONTROLLER_KD = 0.0;

  public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(Pounds.of(ROBOT_MASS_LBS))
          .withCustomModuleTranslations(
              new Translation2d[] {
                FL_MODULE_TRANSLATION,
                FR_MODULE_TRANSLATION,
                BL_MODULE_TRANSLATION,
                BR_MODULE_TRANSLATION
              })
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  DCMotor.getKrakenX60(1),
                  DCMotor.getFalcon500(1),
                  FL_MODULE_CONSTANTS.DriveMotorGearRatio,
                  FL_MODULE_CONSTANTS.SteerMotorGearRatio,
                  Volts.of(FL_MODULE_CONSTANTS.DriveFrictionVoltage),
                  Volts.of(FL_MODULE_CONSTANTS.SteerFrictionVoltage),
                  Meters.of(FL_MODULE_CONSTANTS.WheelRadius),
                  KilogramSquareMeters.of(FL_MODULE_CONSTANTS.SteerInertia),
                  WHEEL_COF));

  //   public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG =
  //       DriveTrainSimulationConfig.Default()
  //           .withRobotMass(Pounds.of(ROBOT_MASS_LBS))
  //           .withCustomModuleTranslations(
  //               new Translation2d[] {
  //                 new Translation2d(
  //                     TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
  //                 new Translation2d(
  //                     TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
  //                 new Translation2d(
  //                     TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
  //                 new Translation2d(
  //                     TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
  //               })
  //           .withGyro(COTS.ofPigeon2())
  //           .withSwerveModule(
  //               new SwerveModuleSimulationConfig(
  //                   DCMotor.getKrakenX60(1),
  //                   DCMotor.getFalcon500(1),
  //                   TunerConstants.FrontLeft.DriveMotorGearRatio,
  //                   TunerConstants.FrontLeft.SteerMotorGearRatio,
  //                   Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
  //                   Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
  //                   Meters.of(TunerConstants.FrontLeft.WheelRadius),
  //                   KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
  //                   WHEEL_COF));
}
