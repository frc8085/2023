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
public class NOTUSEDAutoDynamicDuoMoveOnChargeStation extends SequentialCommandGroup {
  public NOTUSEDAutoDynamicDuoMoveOnChargeStation(
      DriveSubsystem m_drive) {
    addCommands(
        moveOnChargeStation(m_drive));
  }

  // THIS PATH IS ALREADY IN MOVE TO CHARGE STATION
  public Command moveOnChargeStation(DriveSubsystem m_drive) {
    // Create config for trajectory
    TrajectoryConfig config = AutoTrajectoryCommand.config(true);
    int sign = Autos.getAlliance() == Alliance.RED ? 1 : -1;

    // An example trajectory to follow. All units in meters.
    // Should the points be negative or positive? Does it decide based on the
    // reversed being true?
    Trajectory goOnChargeStation = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0.5, sign * 1.8, Rotation2d.fromDegrees(sign * 120)),
        // NOTE: MUST have a waypoint. CANNOT be a straight line.
        List.of(new Translation2d(2, sign * 1.9)),
        // Drive backwards for a meter
        new Pose2d(3, sign * 2, Rotation2d.fromDegrees(sign * 120)),
        config);

    return AutoTrajectoryCommand.command(m_drive, goOnChargeStation);
  }
}