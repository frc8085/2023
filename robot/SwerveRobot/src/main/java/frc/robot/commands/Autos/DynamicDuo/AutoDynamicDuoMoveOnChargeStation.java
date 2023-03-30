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
public class AutoDynamicDuoMoveOnChargeStation extends SequentialCommandGroup {
  public AutoDynamicDuoMoveOnChargeStation(
      DriveSubsystem m_drive) {
    addCommands(
        moveOnChargeStation(m_drive));
  }

  /**
   * could we do something like this?
   * Starting point
   * R1x = 0.5;
   * R1y = 1.8;
   * R1h = 120;
   * R2x = 2;
   * R2y = 1.9;
   * R3x = 3;
   * R3y = 2;
   * R3h = 120;
   * B1x = 0.5;
   * B1y = -1.8;
   * B1h = -120;
   * B2x = 2;
   * B2y = -1.9;
   * B3x = 3;
   * B3y = -2;
   * B3h = -120;
   * 
   */

  public Command moveOnChargeStation(DriveSubsystem m_drive) {
    // Create config for trajectory
    TrajectoryConfig config = AutoTrajectoryCommand.config(true);

    // An example trajectory to follow. All units in meters.
    // Should the points be negative or positive? Does it decide based on the
    // reversed being true?
    Trajectory goOnChargeStation = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0.5, 1.8, Rotation2d.fromDegrees(120)),
        // NOTE: MUST have a waypoint. CANNOT be a straight line.
        List.of(new Translation2d(2, 1.9)),
        // Drive backwards for a meter
        new Pose2d(3, 2, Rotation2d.fromDegrees(120)),
        config);

    return AutoTrajectoryCommand.command(m_drive, goOnChargeStation);
  }
}