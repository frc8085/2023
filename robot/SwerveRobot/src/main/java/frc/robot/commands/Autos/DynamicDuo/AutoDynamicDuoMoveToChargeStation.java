// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.DynamicDuo;

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
import frc.robot.commands.MoveToTravelAfterIntake;
import frc.robot.commands.Autos.Shared.AutoTrajectoryCommand;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class AutoDynamicDuoMoveToChargeStation extends SequentialCommandGroup {
  public AutoDynamicDuoMoveToChargeStation(
      DriveSubsystem m_drive) {
    addCommands(
        moveToChargeStation(m_drive));
  }

  public Command moveToChargeStation(DriveSubsystem m_drive) {
    // Create config for trajectory
    TrajectoryConfig config = AutoTrajectoryCommand.config(true);

    // An example trajectory to follow. All units in meters.
    // Should the points be negative or positive? Does it decide based on the
    // reversed being true?
    Trajectory goToChargeStation = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0.3, .3, Rotation2d.fromDegrees(-180)),
        // NOTE: MUST have a waypoint. CANNOT be a straight line.
        List.of(new Translation2d(0.5, 1.8)),
        // Drive backwards for a meter
        new Pose2d(3, 2, Rotation2d.fromDegrees(90)),
        config);

    return AutoTrajectoryCommand.command(m_drive, goToChargeStation);
  }
}