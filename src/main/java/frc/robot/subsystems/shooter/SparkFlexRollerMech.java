package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.mw_lib.mechanisms.MechBase;

public class SparkFlexRollerMech extends MechBase {

    protected enum ControlMode {
        POSITION,
        VELOCITY,
        DUTY_CYCLE
    }

    private ControlMode control_mode_ = ControlMode.DUTY_CYCLE;

    private final SparkFlex motor_;
    private final SparkClosedLoopController controller_;
    private final SparkFlexConfig stored_config_; // Store our configuration to avoid overwriting

    // sensor inputs
    protected double position_ = 0;
    protected double position_target_ = 0;
    protected double velocity_ = 0;
    protected double velocity_target_ = 0;
    protected double duty_cycle_target_ = 0;
    protected double applied_voltage_ = 0;
    protected double current_draw_ = 0;
    protected double motor_temp_c_ = 0;
    protected double bus_voltage_ = 0;

    // Simulation info
    protected final DCMotorSim roller_sim_;
    protected final double gear_ratio_;

    public SparkFlexRollerMech(double gear_ratio, int can_id, boolean is_inverted) {
        super();

        // Setup motor controller
        motor_ = new SparkFlex(can_id, MotorType.kBrushless);
        stored_config_ = new SparkFlexConfig();
        stored_config_.inverted(is_inverted);
        motor_.configure(stored_config_, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        controller_ = motor_.getClosedLoopController();

        // set the system constants
        this.gear_ratio_ = gear_ratio;

        // default the inputs
        position_ = 0;
        velocity_ = 0;
        applied_voltage_ = 0;
        current_draw_ = 0;
        motor_temp_c_ = 0;
        bus_voltage_ = 0;

        ////////////////////////
        /// SIMULATION SETUP ///
        ////////////////////////
        DCMotor motor_type = DCMotor.getNeoVortex(1);
        roller_sim_ = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(motor_type, 0.01, gear_ratio_), motor_type);
    }

    @Override
    public void readInputs(double timestamp) {
        // always read the sensor data
        position_ = Units.rotationsToRadians(motor_.getAbsoluteEncoder().getPosition());
        velocity_ = Units.rotationsPerMinuteToRadiansPerSecond(motor_.getAbsoluteEncoder().getVelocity());
        applied_voltage_ = motor_.getAppliedOutput();
        current_draw_ = motor_.getOutputCurrent();
        motor_temp_c_ = motor_.getMotorTemperature();
        bus_voltage_ = motor_.getBusVoltage();

        // run the simulation update step here if we are simulating
        if (IS_SIM) {

            SparkFlexSim flex_sim = new SparkFlexSim(motor_, DCMotor.getNeoVortex(1));
            // Set input voltage from motor controller to simulation
            roller_sim_.setInput(applied_voltage_);

            // Update simulation by 20ms
            roller_sim_.update(0.020);
            flex_sim.iterate(Units.radiansPerSecondToRotationsPerMinute(roller_sim_.getAngularVelocityRadPerSec()), 12.0, 0.02);

            // Update simulated sensor readings
            position_ = Units.rotationsToRadians(flex_sim.getAbsoluteEncoderSim().getPosition());
            velocity_ = Units.rotationsPerMinuteToRadiansPerSecond(flex_sim.getAbsoluteEncoderSim().getVelocity());
            applied_voltage_ = flex_sim.getAppliedOutput();
            current_draw_ = roller_sim_.getCurrentDrawAmps();
            bus_voltage_ = flex_sim.getBusVoltage();
        }
    }

    @Override
    public void writeOutputs(double timestamp) {
        switch (control_mode_) {
            case POSITION:
                controller_.setReference(Units.radiansToRotations(position_target_), ControlType.kPosition, ClosedLoopSlot.kSlot0);
                break;
            case VELOCITY:
                controller_.setReference(Units.radiansPerSecondToRotationsPerMinute(velocity_target_), ControlType.kVelocity, ClosedLoopSlot.kSlot1);
                break;
            case DUTY_CYCLE:
                motor_.set(duty_cycle_target_);
                break;
            default:
                throw new IllegalStateException("Unexpected control mode: " + control_mode_);
        }
    }

    /**
     * Configure PID gains for position control (Slot 0)
     * This method preserves all other motor settings by using the stored configuration.
     */
    public void setPositionPID(SlotConfigs config) {
        // Update only the closed-loop parameters in our stored config
        stored_config_.closedLoop.pidf(config.kP, config.kI, config.kD, config.kV, ClosedLoopSlot.kSlot0);
        motor_.configure(stored_config_, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Configure PID gains for velocity control (Slot 1)
     * This method preserves all other motor settings by using the stored configuration.
     */
    public void setVelocityPID(SlotConfigs config) {
        // Update only the closed-loop parameters in our stored config
        stored_config_.closedLoop.pidf(config.kP, config.kI, config.kD, config.kV, ClosedLoopSlot.kSlot1);
        motor_.configure(stored_config_, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setCurrentPosition(double position_rad) {
        // Note: SparkFlex absolute encoder position cannot be set directly
        // This would typically be done during initialization or with a relative encoder
        // For now, we'll store the position offset
        position_ = position_rad;
    }

    /**
     * Get the current position of the roller in radians
     * @return Current position in radians
     */
    public double getCurrentPosition() {
        return position_;
    }

    /**
     * Get the current velocity of the roller in radians per second
     * @return Current velocity in radians per second
     */
    public double getCurrentVelocity() {
        return velocity_;
    }

    /**
     * Get the current draw of the roller motor in amps
     * @return Current draw in amps
     */
    public double getLeaderCurrent() {
        return current_draw_;
    }

    /**
     * Set the target position for the roller in radians
     * @param position_rad Target position in radians
     */
    public void setTargetPosition(double position_rad) {
        position_target_ = position_rad;
        control_mode_ = ControlMode.POSITION;
    }

    /**
     * Set the target velocity for the roller in radians per second
     * @param velocity_rad_per_sec Target velocity in radians per second
     */
    public void setTargetVelocity(double velocity_rad_per_sec) {
        velocity_target_ = velocity_rad_per_sec;
        control_mode_ = ControlMode.VELOCITY;
    }

    /**
     * Set the target velocity for the roller in radians per second
     * @param velocity_rad_per_sec Target velocity in radians per second
     */
    public void setTargetDutyCycle(double duty_cycle) {
        duty_cycle_target_ = duty_cycle;
        control_mode_ = ControlMode.DUTY_CYCLE;
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

        // motor data
        DogLog.log(getLoggingKey() + "motor/applied_voltage", applied_voltage_);
        DogLog.log(getLoggingKey() + "motor/current_draw", current_draw_);
        DogLog.log(getLoggingKey() + "motor/temp_c", motor_temp_c_);
        DogLog.log(getLoggingKey() + "motor/bus_voltage", bus_voltage_);
    }

}
