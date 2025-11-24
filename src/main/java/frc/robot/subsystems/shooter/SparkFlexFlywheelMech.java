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
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.mw_lib.mechanisms.MechBase;

public class SparkFlexFlywheelMech extends MechBase {

    // control modes for the flywheel mechanism
    protected enum ControlMode {
        VELOCITY,
        DUTY_CYCLE
    }

    private ControlMode control_mode_ = ControlMode.DUTY_CYCLE;

    private final SparkFlex motor_;
    private final SparkClosedLoopController controller_;
    private final SparkFlexConfig stored_config_; // Store our configuration to avoid overwriting

    // Simulation info
    protected final FlywheelSim flywheel_sim_;
    protected final double gear_ratio_;
    protected final double wheel_inertia_;
    protected final double wheel_radius_;

    // Current state info
    protected double position_ = 0; // only used in sim
    protected double velocity_ = 0;
    protected double velocity_target_ = 0;
    protected double duty_cycle_target_ = 0;
    protected double applied_voltage_ = 0;
    protected double current_draw_ = 0;
    protected double motor_temp_c_ = 0;
    protected double bus_voltage_ = 0;

    public SparkFlexFlywheelMech(double gear_ratio, double wheel_inertia, double wheel_radius, int can_id, boolean is_inverted) {
        super();

        // Setup motor controller
        motor_ = new SparkFlex(can_id, MotorType.kBrushless);
        stored_config_ = new SparkFlexConfig();
        stored_config_.inverted(is_inverted);
        motor_.configure(stored_config_, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        controller_ = motor_.getClosedLoopController();

        // set the system constants
        this.gear_ratio_ = gear_ratio;
        this.wheel_inertia_ = wheel_inertia;
        this.wheel_radius_ = wheel_radius;

        ////////////////////////
        /// SIMULATION SETUP ///
        ////////////////////////
        DCMotor motor_type = DCMotor.getNeoVortex(1);
        flywheel_sim_ = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(motor_type, wheel_inertia_, gear_ratio_), motor_type);
    }

    @Override
    public void readInputs(double timestamp) {

        // always read the sensor data
        velocity_ = Units.rotationsPerMinuteToRadiansPerSecond(motor_.getAbsoluteEncoder().getVelocity());
        applied_voltage_ = motor_.getAppliedOutput();
        current_draw_ = motor_.getOutputCurrent();
        motor_temp_c_ = motor_.getMotorTemperature();
        bus_voltage_ = motor_.getBusVoltage();

        // run the simulation update step here if we are simulating
        if (IS_SIM) {

            SparkFlexSim flex_sim = new SparkFlexSim(motor_, DCMotor.getNeoVortex(1));
            // Set input voltage from motor controller to simulation
            flywheel_sim_.setInput(applied_voltage_);

            // Update simulation by 20ms
            flywheel_sim_.update(0.020);
            flex_sim.iterate(flywheel_sim_.getAngularVelocityRPM(), 12.0, 0.02);

            // Update simulated sensor readings
            position_ = Units.rotationsToRadians(flex_sim.getAbsoluteEncoderSim().getPosition());
            velocity_ = Units.rotationsPerMinuteToRadiansPerSecond(flex_sim.getAbsoluteEncoderSim().getVelocity());
            applied_voltage_ = flex_sim.getAppliedOutput();
            current_draw_ = flywheel_sim_.getCurrentDrawAmps();
            bus_voltage_ = flex_sim.getBusVoltage();
        }
    }

    @Override
    public void writeOutputs(double timestamp) {
        switch (control_mode_) {
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
     * Configure PID gains for velocity control (Slot 1)
     * This method preserves all other motor settings by using the stored configuration.
     */
    public void setVelocityPID(SlotConfigs config) {
        // Update only the closed-loop parameters in our stored config
        stored_config_.closedLoop.pidf(config.kP, config.kI, config.kD, config.kV, ClosedLoopSlot.kSlot1);
        motor_.configure(stored_config_, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Get the current velocity of the flywheel in radians per second
     * @return Current velocity in radians per second
     */
    public double getCurrentVelocity() {
        return velocity_;
    }

    /**
     * Get the current draw of the flywheel motor in amps
     * @return Current draw in amps
     */
    public double getCurrent() {
        return current_draw_;
    }

    /**
     * Set the target velocity for the flywheel in radians per second
     * @param velocity_rad_per_sec Target velocity in radians per second
     */
    public void setTargetVelocity(double velocity_rad_per_sec) {
        control_mode_ = ControlMode.VELOCITY;
        velocity_target_ = velocity_rad_per_sec;
    }

    /**
     * Set the target duty cycle for the flywheel motor
     * @param duty_cycle Target duty cycle (-1.0 to 1.0)
     */
    public void setTargetDutyCycle(double duty_cycle) {
        control_mode_ = ControlMode.DUTY_CYCLE;
        duty_cycle_target_ = duty_cycle;
    }

    @Override
    public void logData() {
        // commands
        DogLog.log(getLoggingKey() + "control/mode", control_mode_.toString());
        DogLog.log(getLoggingKey() + "control/velocity/target", velocity_target_);
        DogLog.log(getLoggingKey() + "control/velocity/actual", velocity_);
        DogLog.log(getLoggingKey() + "control/duty_cycle/target", duty_cycle_target_);

        DogLog.log(getLoggingKey() + "motor/applied_voltage", applied_voltage_);
        DogLog.log(getLoggingKey() + "motor/current_draw", current_draw_);
        DogLog.log(getLoggingKey() + "motor/temp_c", motor_temp_c_);
        DogLog.log(getLoggingKey() + "motor/bus_voltage", bus_voltage_);
    }

}
