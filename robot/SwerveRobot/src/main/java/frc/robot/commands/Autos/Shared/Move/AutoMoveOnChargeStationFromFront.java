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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.Autos;
import frc.robot.commands.Autos.Autos.Alliance;
import frc.robot.commands.Autos.Shared.AutoTrajectoryCommand;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;

/** An example command that uses an example subsystem. */
public class AutoMoveOnChargeStationFromFront extends SequentialCommandGroup {
  public AutoMoveOnChargeStationFromFront(
      DriveSubsystem m_drive,
      Altitude m_altitude,
      Extension m_extension) {
    addCommands(
        moveOnChargeStationFromFront(m_drive),
        new RunCommand(m_drive::lock));
  }

  /**
   * could we do something like this?
   * Starting point
   * R1x = 4;
   * R1y = 2.05;
   * R1h = 120;
   * R2x = 2.5;
   * R2y = 2.0;
   * R3x = -0.5;
   * R3y = 2.05;
   * R3h = 120;
   * B1x = 4;
   * B1y = -2.05;
   * B1h = -120;
   * B2x = 2.5;
   * B2y = -2.0;
   * B3x = -0.5;
   * B3y = -2.05;
   * B3h = -120;
   * 
   */

  public Command moveOnChargeStationFromFront(DriveSubsystem m_drive) {
    // Create config for trajectory
    TrajectoryConfig config = AutoTrajectoryCommand.config(true);
    int sign = Autos.getAlliance() == Alliance.RED ? 1 : -1;

    // First trajectory. All units in meters.
    Trajectory moveOnChargeStation = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(4, sign * 2.05, Rotation2d.fromDegrees(sign * 120)),
        // Pass through these two interior waypoints, making an 's' curve path
        // NOTE: MUST have a waypoint. CANNOT be a straight line.
        List.of(new Translation2d(2.5, sign * 2.0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(-0.5, sign * 2.05, Rotation2d.fromDegrees(sign * 120)),
        config);

    return AutoTrajectoryCommand.command(m_drive, moveOnChargeStation);
  }
}
