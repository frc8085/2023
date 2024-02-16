// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Shared.Move;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AltitudeConstants;
import frc.robot.Constants.ExtensionConstants;
import frc.robot.commands.Extend;
import frc.robot.commands.RaiseLower;
import frc.robot.commands.Autos.Autos;
import frc.robot.commands.Autos.Autos.Alliance;
import frc.robot.commands.Autos.Shared.AutoTrajectoryCommand;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;

/** An example command that uses an example subsystem. */
public class AutoTravelToThirdPickupAndStartLoweringIntakeClean extends SequentialCommandGroup {
  public AutoTravelToThirdPickupAndStartLoweringIntakeClean(
      DriveSubsystem m_drive,
      Altitude m_altitude,
      Extension m_extension) {
    addCommands(
        new ParallelCommandGroup(
            // new RaiseLower(m_altitude, AltitudeConstants.kAltitudeAutoIntakePosition),
            // new Extend(m_extension, ExtensionConstants.kExtensionPositionIntakeOut),
            travelBackwardsThenSpin(m_drive),
            new SequentialCommandGroup(
                new WaitCommand(1),
                new AutoHMMoveToIntake(m_extension, m_altitude))));
  }

  public Command travelBackwardsThenSpin(DriveSubsystem m_drive) {
    // Create config for trajectory
    TrajectoryConfig config = AutoTrajectoryCommand.config(true);
    int sign = Autos.getAlliance() == Alliance.RED ? 1 : -1;

    // First trajectory. All units in meters.
    Trajectory moveToThirdPosition = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(1.5, sign * 0, Rotation2d.fromDegrees(sign * 178)),
        // Pass through these two interior waypoints, making an 's' curve path
        // NOTE: MUST have a waypoint. CANNOT be a straight line.
        List.of(new Translation2d(2.5, sign * -0.2)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(5, sign * 0.1, Rotation2d.fromDegrees(sign * 178)),
        config);

    return AutoTrajectoryCommand.command(m_drive, moveToThirdPosition);
  }
}