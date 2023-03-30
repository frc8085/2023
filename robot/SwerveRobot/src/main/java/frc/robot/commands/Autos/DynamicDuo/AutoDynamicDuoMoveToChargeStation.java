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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.Autos;
import frc.robot.commands.Autos.Autos.Alliance;
import frc.robot.commands.Autos.Shared.AutoTrajectoryCommand;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class AutoDynamicDuoMoveToChargeStation extends SequentialCommandGroup {
  public AutoDynamicDuoMoveToChargeStation(
      DriveSubsystem m_drive) {
    addCommands(
        moveToChargeStation(m_drive));
  }

  /**
   * could we do something like this?
   * Starting point
   * R1x = 0.3;
   * R1y = 0.3;
   * R1h = -180;
   * R2x = 0.5;
   * R2y = 1.8;
   * R3x = 3;
   * R3y = 2;
   * R3h = 120;
   * B1x = 0.3;
   * B1y = -0.3;
   * B1h = 180;
   * B2x = 0.5;
   * B2y = -1.8;
   * B3x = 3;
   * B3y = -2;
   * B3h = -120;
   * 
   */

  public Command moveToChargeStation(DriveSubsystem m_drive) {
    // Create config for trajectory
    TrajectoryConfig config = AutoTrajectoryCommand.config(true);
    int sign = Autos.getAlliance() == Alliance.RED ? 1 : -1;

    // An example trajectory to follow. All units in meters.
    // Should the points be negative or positive? Does it decide based on the
    // reversed being true?
    Trajectory goToChargeStation = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0.3, sign * .3, Rotation2d.fromDegrees(sign * -180)),
        // NOTE: MUST have a waypoint. CANNOT be a straight line.
        List.of(new Translation2d(0.5, sign * 1.8)),
        // Drive backwards for a meter
        new Pose2d(3, sign * 2, Rotation2d.fromDegrees(sign * 120)),
        config);

    return AutoTrajectoryCommand.command(m_drive, goToChargeStation);
  }
}