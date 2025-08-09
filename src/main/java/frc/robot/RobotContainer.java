// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.mw_lib.subsystem.SubsystemManager;
import frc.robot.subsystems.pose_estimator.PoseEstimator;

public class RobotContainer extends SubsystemManager {
  private static RobotContainer instance_;

  public static synchronized RobotContainer getInstance() {
    if (instance_ == null) {
      instance_ = new RobotContainer();
    }
    return instance_;
  }

  public RobotContainer() {
    // !!!!!! ALL SUBSYSTEMS MUST BE REGISTERED HERE TO RUN !!!!!!!
    // registerSubsystem(Swerve.getInstance());
    registerSubsystem(PoseEstimator.getInstance());

    // !!!!! LEAVE THESE LINES AS THE LAST LINE IN THE CONSTRUCTOR !!!!!!
    reset();
    completeRegistration();
  }
}
