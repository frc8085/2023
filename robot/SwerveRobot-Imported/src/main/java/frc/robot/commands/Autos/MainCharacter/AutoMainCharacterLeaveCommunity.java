// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.MainCharacter;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.Shared.AutoTrajectoryCommand;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;

/** An example command that uses an example subsystem. */
public class AutoMainCharacterLeaveCommunity extends SequentialCommandGroup {
    public AutoMainCharacterLeaveCommunity(
            DriveSubsystem m_drive,
            Altitude m_altitude,
            Extension m_extension) {
        addCommands(
                travelToLeaveCommunity(m_drive));

    }

    public Command travelToLeaveCommunity(DriveSubsystem m_drive) {
        // Create config for trajectory
        TrajectoryConfig config = AutoTrajectoryCommand.config(true);

        // First trajectory. All units in meters.
        Trajectory leaveCommunity = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, Rotation2d.fromDegrees(-180)),
                // Pass through these two interior waypoints, making an 's' curve path
                // NOTE: MUST have a waypoint. CANNOT be a straight line.
                List.of(new Translation2d(2, -0.01)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(4, -0.1, Rotation2d.fromDegrees(0)),
                config);

        return AutoTrajectoryCommand.command(m_drive, leaveCommunity);
    }
}
