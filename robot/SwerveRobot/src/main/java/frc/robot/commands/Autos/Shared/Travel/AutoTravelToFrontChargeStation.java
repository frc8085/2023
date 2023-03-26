// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Shared.Travel;

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
import frc.robot.FieldLandmarks;
import frc.robot.commands.MoveToTravelAfterIntake;
import frc.robot.commands.Autos.Shared.AutoTrajectoryCommand;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class AutoTravelToFrontChargeStation extends SequentialCommandGroup {
    public AutoTravelToFrontChargeStation(
            DriveSubsystem m_drive,
            Altitude m_altitude,
            Extension m_extension,
            Intake m_intake) {
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(() -> m_intake.holdCargo()),
                        new MoveToTravelAfterIntake(m_extension, m_altitude)),
                travelToChargeStation(m_drive));
    }

    public Command travelToChargeStation(DriveSubsystem m_drive) {
        // Create config for trajectory
        TrajectoryConfig config = AutoTrajectoryCommand.config(true);

        Trajectory moveToChargeStation = TrajectoryGenerator.generateTrajectory(
                FieldLandmarks.PickupPosition.Blue1,
                // NOTE: MUST have a waypoint. CANNOT be a straight line.
                List.of(FieldLandmarks.InteriorWaypoint.HaflwayToStation),
                FieldLandmarks.SegmentEndpoints.ReachCharingStation,
                config);

        return AutoTrajectoryCommand.command(m_drive, moveToChargeStation);
    }
}
