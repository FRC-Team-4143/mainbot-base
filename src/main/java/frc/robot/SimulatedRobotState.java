package frc.robot;

import frc.mw_lib.simulation.ArenaEvergreen;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;

public class SimulatedRobotState {
    private static SwerveDriveSimulation swerve_simulation_;

    public static void configure() {

        // Add the swerve drive simulation to the simulated arena
        swerve_simulation_ = SwerveSubsystem.getInstance().getSwerveSimulation();
        SimulatedArena.overrideInstance(new ArenaEvergreen());
        SimulatedArena.getInstance().addDriveTrainSimulation(swerve_simulation_);
        SimulatedArena.getInstance().resetFieldForAuto();
    }
}
