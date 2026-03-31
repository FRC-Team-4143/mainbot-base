// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.Optional;

public class Robot extends TimedRobot {

    private Alliance alliance_ = Alliance.Blue; // Current alliance, used to set driver perspective
    private RobotContainer robot_container_;

    public Robot() {

        // Load the subsystems
        robot_container_ = RobotContainer.getInstance();

        // Configure External Interfaces
        OI.configureBindings();
    }

    @Override
    public void robotInit() {}

    @Override
    public void robotPeriodic() {
        // Call the scheduler so that commands work for buttons
        // run the main robot loop for each subsystem
        robot_container_.doControlLoop();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        // Allow chaning alliance perspective while disabled
        if (hasAllianceChanged()) {
            SwerveSubsystem.getInstance()
                    .setOperatorForwardDirection(
                            alliance_ == Alliance.Blue
                                    ? SwerveConstants.OperatorPerspective.BLUE_ALLIANCE
                                    : SwerveConstants.OperatorPerspective.RED_ALLIANCE);
        }
    }

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        SwerveSubsystem.getInstance().setWantedState(SwerveStates.FIELD_CENTRIC);
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    /**
     * Check if the alliance has changed since the last check
     *
     * @return true if the alliance has changed, false otherwise
     */
    public boolean hasAllianceChanged() {
        Optional<Alliance> current_alliance = DriverStation.getAlliance();
        if (alliance_ == null && current_alliance.isPresent()) {
            alliance_ = current_alliance.get();
            return true;
        } else if (current_alliance.isPresent() && alliance_ != current_alliance.get()) {
            alliance_ = current_alliance.get();
            return true;
        }
        return false;
    }
}
