package frc.robot.subsystems.drivetrain;

import java.util.Arrays;
import java.util.List;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import frc.mw_lib.subsystem.MwSubsystem;
import frc.mw_lib.subsystem.SubsystemIoBase;
import frc.robot.OI;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.DrivetrainStates;

public class DrivetrainSubsystem extends MwSubsystem<DrivetrainStates, DrivetrainConstants> {
    private static DrivetrainSubsystem instance_ = null;

    public static DrivetrainSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new DrivetrainSubsystem();
        }
        return instance_;
    }

    private DifferentialDriveMech drive_mech_;
    private double fwd_command_ = 0.0;
    private double rot_command_ = 0.0;

    public DrivetrainSubsystem() {
        super(DrivetrainStates.IDLE, new DrivetrainConstants());
        drive_mech_ = new DifferentialDriveMech(
            CONSTANTS.wheel_circumference_meters,
            CONSTANTS.track_width_meters
        );

        drive_mech_.setLoggingPrefix(getSubsystemKey());

    }

    // @Override
    // public void handleStateTransition(ElevatorStates wanted) {
    // }

    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case TELE_OP:
                    fwd_command_ = OI.getDriverJoystickLeftY();
                    rot_command_ = OI.getDriverJoystickRightX();
                break;
            case AUTO:
                    // use fwd_command_ and rot_command_ set by auto command
                break;
            default:
            case IDLE:
                    drive_mech_.stop();
                break;
        }

        // send commands to the drive mechanism
        drive_mech_.arcadeDrive(fwd_command_, rot_command_);

        // log data
        DogLog.log(getSubsystemKey() + "/FwdCmd", fwd_command_);
        DogLog.log(getSubsystemKey() + "/RotCmd", rot_command_);
        DogLog.log(getSubsystemKey() + "/State", system_state_.toString());
        DogLog.log(getSubsystemKey() + "/Pose", getCurrentPosition());
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(drive_mech_);
    }

    @Override
    public void reset() {
        system_state_ = DrivetrainStates.IDLE;
    }

    public void setAutoCommand(double fwd, double rot) {
        setWantedState(DrivetrainStates.AUTO);
        fwd_command_ = fwd;
        rot_command_ = rot;
    }

    public void zeroCurrentPosition() {
        drive_mech_.zeroCurrentPosition();
    }

    public Pose2d getCurrentPosition() {
        return drive_mech_.getCurrentPosition();
    }

}
