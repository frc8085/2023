// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AltitudeConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ExtensionConstants;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/** An example command that uses an example subsystem. */
public class AutoSidekickMoveToTravel extends SequentialCommandGroup {
    public AutoSidekickMoveToTravel(
            DriveSubsystem m_drive,
            Altitude m_altitude,
            Extension m_extension) {
        addCommands(
                // 1. score
                new ParallelCommandGroup(
                        travelBackwardsThenSpin(m_drive),
                        new SequentialCommandGroup(
                                new RaiseLower(m_altitude, AltitudeConstants.kAltitudeIntakePosition),
                                new WaitUntilCommand(() -> m_altitude.AltitudeIsInScoringPosition()),
                                new Extend(m_extension, ExtensionConstants.kExtensionPositionFullyRetracted))));

    }

    public Command travelBackwardsThenSpin(DriveSubsystem m_drive) {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        // Travel backwards through our trajectory
        config.setReversed(true);

        // First trajectory. All units in meters.
        Trajectory trajectoryOne = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, Rotation2d.fromDegrees(-180)),
                // NOTE: MUST have a waypoint. CANNOT be a straight line.
                List.of(new Translation2d(.5, 0.01)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(1, 0, Rotation2d.fromDegrees(-180)),
                config);

        // First trajectory. All units in meters.
        Trajectory trajectoryTwo = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(1, 0, Rotation2d.fromDegrees(-180)),
                // NOTE: MUST have a waypoint. CANNOT be a straight line.
                List.of(new Translation2d(2, 0.01)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
                config);

        var concatTraj = trajectoryOne.concatenate(trajectoryTwo);

        var thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                concatTraj,
                m_drive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_drive::setModuleStates,
                m_drive);

        // Reset odometry to the starting pose of the trajectory.
        m_drive.resetOdometry(concatTraj.getInitialPose());
        m_drive.zeroHeading();

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_drive.stop());
    }
}
