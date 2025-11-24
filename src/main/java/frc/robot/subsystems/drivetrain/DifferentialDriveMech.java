package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import dev.doglog.DogLog;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import frc.mw_lib.mechanisms.MechBase;

public class DifferentialDriveMech extends MechBase {

    protected enum ControlMode {
        IDLE,
        DUTY_CYCLE
    }

    private ControlMode control_mode_ = ControlMode.IDLE;

    // Setup Hardware Devices
    private final TalonSRX fl_drive_motor_;
    private final TalonSRX fr_drive_motor_;
    private final TalonSRX bl_drive_motor_;
    private final TalonSRX br_drive_motor_;
    private final TalonSRX[] motors_;
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
    protected double left_duty_cycle_target_ = 0;
    protected double right_duty_cycle_target_ = 0;
    protected Rotation2d yaw_ = Rotation2d.kZero;

    public DifferentialDriveMech(double track_width_meters, double wheel_diameter_meters) {
        super();

        // Initialize motor controllers first
        fl_drive_motor_ = new TalonSRX(1);
        fr_drive_motor_ = new TalonSRX(2);
        bl_drive_motor_ = new TalonSRX(3);
        br_drive_motor_ = new TalonSRX(4);
        motors_ = new TalonSRX[] {fl_drive_motor_, fr_drive_motor_, bl_drive_motor_, br_drive_motor_};

        // Set followers
        bl_drive_motor_.follow(fl_drive_motor_);
        br_drive_motor_.follow(fr_drive_motor_);

        // Invert motors
        fl_drive_motor_.setInverted(false);
        bl_drive_motor_.setInverted(false);
        fr_drive_motor_.setInverted(true);
        br_drive_motor_.setInverted(true);

        // Configure encoders
        fl_drive_motor_.setSensorPhase(true);
        fr_drive_motor_.setSensorPhase(true);
        fl_drive_motor_.setSelectedSensorPosition(0);
        fr_drive_motor_.setSelectedSensorPosition(0);

        // Now create DifferentialDrive with DoubleConsumer lambdas
        diff_drive = new DifferentialDrive(
            (speed) -> left_duty_cycle_target_ = speed,
            (speed) -> right_duty_cycle_target_ = speed
        );

        imu_ = new AHRS(NavXComType.kMXP_SPI);

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

        // Ensure IMU is calibrated before setting zero offset
        if(!IS_SIM){
            while(imu_.isCalibrating()){
                Timer.delay(0.02);
            }
            imu_.zeroYaw();
        }
    }

    @Override
    public void readInputs(double timestamp) {

        // always read the sensor data
        left_position_ = fl_drive_motor_.getSelectedSensorPosition() / 4096.0 * wheel_circumference_meters_;
        right_position_ = fr_drive_motor_.getSelectedSensorPosition() / 4096.0 * wheel_circumference_meters_;
        left_velocity_ = fl_drive_motor_.getSelectedSensorVelocity() / 4096.0 * wheel_circumference_meters_ * 10.0;
        right_velocity_ = fr_drive_motor_.getSelectedSensorVelocity() / 4096.0 * wheel_circumference_meters_ * 10.0;

        yaw_ = Rotation2d.fromDegrees(-imu_.getYaw());

        // run the simulation update step here if we are simulating
        if (IS_SIM) {
            drive_sim_.setInputs(left_duty_cycle_target_ * 12.0, right_duty_cycle_target_* 12.0);
            drive_sim_.update(0.02);

            // Update simulated sensor readings
            left_position_ = drive_sim_.getLeftPositionMeters();
            right_position_ = drive_sim_.getRightPositionMeters();
            left_velocity_ = drive_sim_.getLeftVelocityMetersPerSecond();
            right_velocity_ = drive_sim_.getRightVelocityMetersPerSecond();
            yaw_ = drive_sim_.getHeading();
        }

        // Update pose estimator
        pose_estimator_.update(yaw_, left_position_, right_position_);
    }

    @Override
    public void writeOutputs(double timestamp) {
        switch (control_mode_) {
            case DUTY_CYCLE:
                fl_drive_motor_.set(TalonSRXControlMode.PercentOutput, left_duty_cycle_target_);
                fr_drive_motor_.set(TalonSRXControlMode.PercentOutput, right_duty_cycle_target_);
                break;
            case IDLE:
                fl_drive_motor_.set(TalonSRXControlMode.PercentOutput, 0.0);
                fr_drive_motor_.set(TalonSRXControlMode.PercentOutput, 0.0);
                break;
            default:
                throw new IllegalStateException("Unexpected control mode: " + control_mode_);
        }
    }

    /**
     * Sets the current position of the drivetrain to a zero Pose2d.
     */
    public void zeroCurrentPosition() {
        fl_drive_motor_.setSelectedSensorPosition(0);
        fr_drive_motor_.setSelectedSensorPosition(0);
        imu_.zeroYaw();
        //pose_estimator_.resetPose(Pose2d.kZero);

        if(IS_SIM){
            drive_sim_.setPose(Pose2d.kZero);
        }
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        control_mode_ = ControlMode.DUTY_CYCLE;
        diff_drive.arcadeDrive(fwd, rot);
    }

    /**
     * Drives the robot using tank controls.
     *
     * @param left the commanded left side movement
     * @param right the commanded right side movement
     */
    public void tankDrive(double left, double right) {
        control_mode_ = ControlMode.DUTY_CYCLE;
        diff_drive.tankDrive(left, right);
    }

    /**
     * Drives the robot using curvature controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void curvatureDrive(double fwd, double rot) {
        control_mode_ = ControlMode.DUTY_CYCLE;
        diff_drive.curvatureDrive(fwd, rot, true);
    }

    public void stop() {
        control_mode_ = ControlMode.IDLE;
        diff_drive.stopMotor();
    }

    /**
     * @return the estimated position of the robot on the field
     */
    public Pose2d getCurrentPosition() {
        return pose_estimator_.getEstimatedPosition();
    }

    /**
     * @return the current chassis speeds of the robot
     */
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics_.toChassisSpeeds(new DifferentialDriveWheelSpeeds(left_velocity_, right_velocity_));
    }

    @Override
    public void logData() {
        // commands
        DogLog.log(getLoggingKey() + "control/mode", control_mode_.toString());
        DogLog.log(getLoggingKey() + "control/position/target", position_target_);
        DogLog.log(getLoggingKey() + "control/position/left_actual", left_position_);
        DogLog.log(getLoggingKey() + "control/velocity/left_actual", left_velocity_);
        DogLog.log(getLoggingKey() + "control/position/right_actual", right_position_);
        DogLog.log(getLoggingKey() + "control/velocity/right_actual", right_velocity_);
        DogLog.log(getLoggingKey() + "control/duty_cycle/left_target", left_duty_cycle_target_);
        DogLog.log(getLoggingKey() + "control/duty_cycle/right_target", right_duty_cycle_target_);

        // imu
        DogLog.log(getLoggingKey() + "imu/yaw_deg", yaw_.getDegrees());
    }

}
