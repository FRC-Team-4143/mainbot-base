package frc.robot;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class SimulatedRobotState {
    private static SwerveDriveSimulation swerve_simulation_;

    public static void configure() {
        swerve_simulation_ = SwerveSubsystem.getInstance().getSwerveSimulation();
    }
}
