package frc.robot;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SimulatedRobotState {
    private static SwerveDriveSimulation swerve_simulation_;

    public static void configure(){
        swerve_simulation_ = SwerveSubsystem.getInstance().getSwerveSimulation();
    }
}
