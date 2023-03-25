// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Sidekick;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.MoveToIntake;
import frc.robot.commands.Autos.Shared.AutoTrajectoryCommand;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class AutoSidekickPickupCargo extends SequentialCommandGroup {
  public AutoSidekickPickupCargo(
      DriveSubsystem m_drive,
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    addCommands(
        new MoveToIntake(m_extension, m_altitude),
        new WaitUntilCommand(
            () -> m_altitude.AltitudeIsInIntakePosition() &&
                m_extension.ExtensionIsInIntakePosition()),
        new ParallelCommandGroup(
            driveToGamePiece(m_drive),
            new InstantCommand(() -> m_intake.intakeCone())));
  }

  public Command driveToGamePiece(DriveSubsystem m_drive) {
    // Create config for trajectory
    TrajectoryConfig config = AutoTrajectoryCommand.config(false);

    // An example trajectory to follow. All units in meters.
    Trajectory pickupCargo = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing forward
        new Pose2d(3, -0.1, Rotation2d.fromDegrees(-10)),
        // NOTE: MUST have a waypoint. CANNOT be a straight line.
        List.of(new Translation2d(3.2, -0.15)),
        // End 2 meters straight ahead of where we started still facing forward
        new Pose2d(3.45, -0.25, Rotation2d.fromDegrees(-10)),
        config);

    return AutoTrajectoryCommand.command(m_drive, pickupCargo);
  }
}