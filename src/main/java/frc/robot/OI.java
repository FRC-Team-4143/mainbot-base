// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.Optional;

public abstract class OI {

    // Sets up both controllers
    private static CommandXboxController driver_controller_ = new CommandXboxController(0);

    public static void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        SmartDashboard.putData(
                "Set Wheel Offsets",
                SwerveSubsystem.getInstance().setModuleOffsets().ignoringDisable(true));
        SmartDashboard.putData(
                "Zero Gyro", SwerveSubsystem.getInstance().zeroGyro().ignoringDisable(true));
        SmartDashboard.putData(
                "Toggle Simple Sim Control",
                SwerveSubsystem.getInstance().toggleSimpleSimulationControl());

        driver_controller_.a().onTrue(SwerveSubsystem.getInstance().toggleFieldCentric());
        // Bind the back button to toggle simple simulation control (only works in simulation)
        driver_controller_
                .back()
                .onTrue(SwerveSubsystem.getInstance().toggleSimpleSimulationControl());
    }

    /**
     * @return driver controller left joystick x axis scaled quadratically
     */
    public static double getDriverJoystickLeftX() {
        return driver_controller_.getLeftX();
    }

    /**
     * @return driver controller left joystick y axis scaled quadratically
     */
    public static double getDriverJoystickLeftY() {
        return driver_controller_.getLeftY();
    }

    /**
     * @return driver controller right joystick x axis scaled quadratically
     */
    public static double getDriverJoystickRightX() {
        return driver_controller_.getRightX();
    }

    /**
     * @return driver controller joystick pov angle in degs. empty if nothing is pressed
     */
    public static Optional<Rotation2d> getDriverJoystickPOV() {
        int pov = driver_controller_.getHID().getPOV();
        return (pov != -1) ? Optional.of(Rotation2d.fromDegrees(pov)) : Optional.empty();
    }
}
