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
import frc.robot.commands.Autos.Autos;
import frc.robot.commands.Autos.Autos.Alliance;
import frc.robot.commands.Autos.Shared.AutoTrajectoryCommand;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class AutoDynamicDuoReturnToScore extends SequentialCommandGroup {
  public AutoDynamicDuoReturnToScore(
      DriveSubsystem m_drive,
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    addCommands(
        new ParallelCommandGroup(
            new InstantCommand(() -> m_intake.holdCargo()),
            new MoveToTravelAfterIntake(m_extension, m_altitude)),
        returnToScore(m_drive));
  }

  /**
   * could we do something like this?
   * Starting point
   * R1x = 5.0;
   * R1y = .35;
   * R1h = 5;
   * R2x = 2.5;
   * R2y = 0.3;
   * R3x = 0.3;
   * R3y = 0.3;
   * R3h = -180;
   * B1x = 5.0;
   * B1y = -0.35;
   * B1h = -5;
   * B2x = 2.5;
   * B2y = -0.3;
   * B3x = 0.3;
   * B3y = -0.3;
   * B3h = 180;
   * 
   */

  public Command returnToScore(DriveSubsystem m_drive) {
    // Create config for trajectory
    TrajectoryConfig config = AutoTrajectoryCommand.config(true);
    int sign = Autos.getAlliance() == Alliance.RED ? 1 : -1;

    // An example trajectory to follow. All units in meters.
    // Should the points be negative or positive? Does it decide based on the
    // reversed being true?
    Trajectory returnToScoreOne = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(5.0, sign * .35, Rotation2d.fromDegrees(sign * 5)),
        // NOTE: MUST have a waypoint. CANNOT be a straight line.
        List.of(new Translation2d(2.5, sign * 0.3)),
        // Drive backwards for a meter
        new Pose2d(0.3, sign * 0.3, Rotation2d.fromDegrees(sign * -180)),
        config);

    return AutoTrajectoryCommand.command(m_drive, returnToScoreOne);
  }
}