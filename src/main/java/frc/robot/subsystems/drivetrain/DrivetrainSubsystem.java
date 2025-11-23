package frc.robot.subsystems.drivetrain;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.configs.SlotConfigs;

import dev.doglog.DogLog;
import frc.mw_lib.mechanisms.ElevatorMech;
import frc.mw_lib.subsystem.MwSubsystem;
import frc.mw_lib.subsystem.SubsystemIoBase;
import frc.mw_lib.util.TunablePid;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.DrivetrainStates;

public class DrivetrainSubsystem extends MwSubsystem<DrivetrainStates, DrivetrainConstants> {
    private static DrivetrainSubsystem instance_ = null;

    public static DrivetrainSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new DrivetrainSubsystem();
        }
        return instance_;
    }

    public DrivetrainSubsystem() {
        super(DrivetrainStates.IDLE, new DrivetrainConstants());

    }

    // @Override
    // public void handleStateTransition(ElevatorStates wanted) {

    // }

    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case TELE_OP:
                break;
            case AUTO:
                break;
            default:
            case IDLE:
                break;
        }
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(elevator_mech_);
    }

    @Override
    public void reset() {
        system_state_ = DrivetrainStates.IDLE;
    }

}
