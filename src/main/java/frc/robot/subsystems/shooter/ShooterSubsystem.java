package frc.robot.subsystems.shooter;

import java.util.Arrays;
import java.util.List;

import frc.mw_lib.subsystem.MwSubsystem;
import frc.mw_lib.subsystem.SubsystemIoBase;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;

public class ShooterSubsystem extends MwSubsystem<ShooterStates, ShooterConstants> {
    private static ShooterSubsystem instance_ = null;

    public static ShooterSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new ShooterSubsystem();
        }
        return instance_;
    }

    public ShooterSubsystem() {
        super(ShooterStates.ACTIVE, new ShooterConstants());
    }

    // @Override
    // public void handleStateTransition(ElevatorStates wanted) {
    // }

    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case ACTIVE:
            default:
                // Your code here!
                break;
        }


        // log data
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(null);
    }

    @Override
    public void reset() {
        system_state_ = ShooterStates.ACTIVE;
    }

}
