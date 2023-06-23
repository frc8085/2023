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
import frc.robot.commands.Autos.Shared.AutoTrajectoryVariableSpeedCommand;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class AutoDynamicDuoMoveToThenOnChargeStation extends SequentialCommandGroup {
  public AutoDynamicDuoMoveToThenOnChargeStation(
      DriveSubsystem m_drive) {
    addCommands(
        moveToChargeStation(m_drive));
  }

  public Command moveToChargeStation(DriveSubsystem m_drive) {
    // Create config for trajectory
    TrajectoryConfig config = AutoTrajectoryVariableSpeedCommand.config(true, 2);
    int sign = Autos.getAlliance() == Alliance.RED ? 1 : -1;

    // An example trajectory to follow. All units in meters.
    // Should the points be negative or positive? Does it decide based on the
    // reversed being true?
    Trajectory goToChargeStation = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0.3, sign * .1, Rotation2d.fromDegrees(sign * 178)),
        // NOTE: MUST have a waypoint. CANNOT be a straight line.
        List.of(new Translation2d(1, sign * 1.2)),
        // Drive backwards for a meter
        new Pose2d(4.2, sign * 1.5, Rotation2d.fromDegrees(sign * 30)),
        config);

    return AutoTrajectoryVariableSpeedCommand.command(m_drive, goToChargeStation);
  }
}