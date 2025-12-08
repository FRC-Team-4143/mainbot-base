package frc.robot.subsystems.localization;

import java.util.List;

import frc.mw_lib.subsystem.SubsystemIoBase;
import frc.mw_lib.swerve_lib.PhoenixOdometryThread;
import frc.mw_lib.swerve_lib.SwerveMeasurments.GyroMeasurement;
import frc.mw_lib.swerve_lib.SwerveMeasurments.ModuleMeasurement;

public class LocalizationIO implements SubsystemIoBase {
    List<ModuleMeasurement> module_measurements_ ;
    List<GyroMeasurement> gyro_measurements_;

    public LocalizationIO(){}

    @Override
    public void readInputs(double timestamp) {
        module_measurements_ = PhoenixOdometryThread.getInstance().getModuleSamples(0);
        gyro_measurements_ = PhoenixOdometryThread.getInstance().getGyroSamples();
    }

    @Override
    public void writeOutputs(double timestamp) {
        // No Outputs to Write for this IO
    }

    @Override
    public void logData(){

    }

}
