// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Shared.Move;

import java.lang.reflect.Field;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldLandmarks;
import frc.robot.commands.Autos.Shared.AutoTrajectoryCommand;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;

/** An example command that uses an example subsystem. */
public class AutoMoveToPickup extends SequentialCommandGroup {
  public AutoMoveToPickup(
      DriveSubsystem m_drive,
      Altitude m_altitude,
      Extension m_extension) {
    addCommands(
        travelBackwardsThenSpin(m_drive));

  }

  public Command travelBackwardsThenSpin(DriveSubsystem m_drive) {
    // Create config for trajectory
    TrajectoryConfig config = AutoTrajectoryCommand.config(true);

    // First trajectory. All units in meters.
    Trajectory moveToPosition = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        FieldLandmarks.GridPosition.BlueALeft,
        // Pass through these two interior waypoints, making an 's' curve path
        // NOTE: MUST have a waypoint. CANNOT be a straight line.
        List.of(FieldLandmarks.InteriorWaypoint.HaflwayToPickup),
        // End 3 meters straight ahead of where we started, facing forward
        FieldLandmarks.SegmentEndpoints.ApproachBlue1,
        config);

    m_drive.zeroHeading();
    m_drive.resetOdometry(moveToPosition.getInitialPose());

    return AutoTrajectoryCommand.command(m_drive, moveToPosition);
  }
}
