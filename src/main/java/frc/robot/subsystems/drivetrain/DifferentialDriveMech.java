package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;

import dev.doglog.DogLog;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.mw_lib.mechanisms.FxMotorConfig.FxMotorType;
import frc.mw_lib.mechanisms.MechBase;

public class DifferentialDriveMech extends MechBase {

    protected enum ControlMode {
        MOTION_MAGIC_POSITION,
        POSITION,
        VELOCITY,
        DUTY_CYCLE
    }

    private ControlMode control_mode_ = ControlMode.DUTY_CYCLE;

    // Setup Hardware Devices
    private final TalonSRX fl_drive_motor_;
    private final TalonSRX fr_drive_motor_;
    private final TalonSRX bl_drive_motor_;
    private final TalonSRX br_drive_motor_;
    private final AHRS imu_;

    // Drivetrain Control Objects
    private final DifferentialDrive diff_drive;
    private final DifferentialDriveKinematics kinematics_;
    private final DifferentialDrivePoseEstimator pose_estimator_;
    private final double track_width_meters_;
    private final double wheel_circumference_meters_;

    // Simulation
    private final DifferentialDrivetrainSim drive_sim_;

    // sensor inputs
    protected double left_position_ = 0;
    protected double right_position_ = 0;
    protected double position_target_ = 0;
    protected double left_velocity_ = 0;
    protected double right_velocity_ = 0;
    protected double velocity_target_ = 0;
    protected double duty_cycle_target_ = 0;
    protected double[] applied_voltage_;
    protected double[] current_draw_;
    protected double[] motor_temp_c_;
    protected double[] bus_voltage_;

    public DifferentialDriveMech(double track_width_meters, double wheel_diameter_meters) {
        super();

        // Initialize motor controllers first
        fl_drive_motor_ = new TalonSRX(1);
        fr_drive_motor_ = new TalonSRX(2);
        bl_drive_motor_ = new TalonSRX(3);
        br_drive_motor_ = new TalonSRX(4);

        // Now create DifferentialDrive with DoubleConsumer lambdas
        diff_drive = new DifferentialDrive(
            (speed) -> fl_drive_motor_.set(TalonSRXControlMode.PercentOutput, speed),
            (speed) -> fr_drive_motor_.set(TalonSRXControlMode.PercentOutput, speed)
        );

        this.track_width_meters_ = track_width_meters;
        this.wheel_circumference_meters_ = Math.PI * wheel_diameter_meters;
        kinematics_ = new DifferentialDriveKinematics(Units.inchesToMeters(this.track_width_meters_));
        pose_estimator_ = new DifferentialDrivePoseEstimator(kinematics_, Rotation2d.kZero, 0.0, 0.0, Pose2d.kZero);

        ////////////////////////
        /// SIMULATION SETUP ///
        ////////////////////////

        drive_sim_ = DifferentialDrivetrainSim.createKitbotSim(
            KitbotMotor.kDualCIMPerSide,
            KitbotGearing.k10p71,
            KitbotWheelSize.kSixInch,
            null);

    }

    @Override
    public void readInputs(double timestamp) {

        // always read the sensor data
        left_position_ = fl_drive_motor_.getSelectedSensorPosition() / 4096.0 * wheel_circumference_meters_;
        right_position_ = fr_drive_motor_.getSelectedSensorPosition() / 4096.0 * wheel_circumference_meters_;
        left_velocity_ = fl_drive_motor_.getSelectedSensorVelocity() / 4096.0 * wheel_circumference_meters_ * 10.0;
        right_velocity_ = fr_drive_motor_.getSelectedSensorVelocity() / 4096.0 * wheel_circumference_meters_ * 10.0;

        position_ = motors_[0].getPosition().getValue().in(Radians);
        velocity_ = motors_[0].getVelocity().getValue().in(RadiansPerSecond);
        for (int i = 0; i < motors_.length; i++) {
            applied_voltage_[i] = motors_[i].getMotorVoltage().getValueAsDouble();
            current_draw_[i] = motors_[i].getSupplyCurrent().getValue().in(Amps);
            motor_temp_c_[i] = motors_[i].getDeviceTemp().getValue().in(Celsius);
            bus_voltage_[i] = motors_[i].getSupplyVoltage().getValueAsDouble();
        }

        // run the simulation update step here if we are simulating
        if (IS_SIM) {
            drive_sim_.setInputs(left_applied_percent_ * 12.0, right_applied_percent_* 12.0);
            drive_sim_.update(kDefaultPeriod);

        }
    }

    @Override
    public void writeOutputs(double timestamp) {
        switch (control_mode_) {
            case DUTY_CYCLE:
                diff_drive.arcadeDrive(timestamp, timestamp);
                break;
            default:
                throw new IllegalStateException("Unexpected control mode: " + control_mode_);
        }
    }

    public void setCurrentPosition(Pose2d pose) {
        pose_estimator_.resetPosition(pose.getRotation(), 0.0, 0.0, pose);
        fl_drive_motor_.setSelectedSensorPosition(0.0);
        fr_drive_motor_.setSelectedSensorPosition(0.0);
    }

    public Pose2d getCurrentPosition() {
        return pose_estimator_.getEstimatedPosition();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics_.toChassisSpeeds(new DifferentialDriveWheelSpeeds(left_velocity_, right_velocity_));
    }

    @Override
    public void logData() {
        // commands
        DogLog.log(getLoggingKey() + "control/mode", control_mode_.toString());
        DogLog.log(getLoggingKey() + "control/position/target", position_target_);
        DogLog.log(getLoggingKey() + "control/position/actual", position_);
        DogLog.log(getLoggingKey() + "control/velocity/target", velocity_target_);
        DogLog.log(getLoggingKey() + "control/velocity/actual", velocity_);
        DogLog.log(getLoggingKey() + "control/duty_cycle/target", duty_cycle_target_);

        // per motor data
        for (int i = 0; i < motors_.length; i++) {
            DogLog.log(getLoggingKey() + "motor" + i + "/applied_voltage", applied_voltage_[i]);
            DogLog.log(getLoggingKey() + "motor" + i + "/current_draw", current_draw_[i]);
            DogLog.log(getLoggingKey() + "motor" + i + "/temp_c", motor_temp_c_[i]);
            DogLog.log(getLoggingKey() + "motor" + i + "/bus_voltage", bus_voltage_[i]);
        }
    }

}
