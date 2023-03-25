// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Shared;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/** A class to generate a final trajectory command from input trajectory **/
public class AutoTrajectoryCommand extends SequentialCommandGroup {

  /**
   * Reusable trajectory config
   *
   * @param reversed Are we driving backward along the trajectory
   */

  public static TrajectoryConfig config(boolean reversed) {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // Whether to travel backwards through our trajectory
    config.setReversed(reversed);

    return config;
  }

  /**
   * Generates a trajectory swerveControllerCommand
   *
   * @param m_drive    The drive sybsystem
   * @param trajectory The trajectory to drive
   */

  public static Command command(DriveSubsystem m_drive, Trajectory trajectory) {
    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        m_drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_drive::setModuleStates,
        m_drive);

    // Reset odometry to the starting pose of the trajectory.
    // m_drive.resetOdometry(trajectory.getInitialPose());
    // m_drive.zeroHeading();

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_drive.stop());
  }
}
