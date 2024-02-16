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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Autos.Autos;
import frc.robot.commands.Autos.Autos.Alliance;
import frc.robot.commands.Autos.Shared.AutoTrajectoryCommand;
import frc.robot.commands.Autos.Shared.AutoTrajectoryVariableSpeedCommand;
import frc.robot.commands.Autos.Shared.Move.AutoMoveToIntake;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

// Used for picking up cargo from the "clean" side of the field
public class AutoMainCharacterPickupConeClean extends SequentialCommandGroup {
  public AutoMainCharacterPickupConeClean(
      DriveSubsystem m_drive,
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    addCommands(
        new AutoMoveToIntake(m_extension, m_altitude),
        new WaitUntilCommand(
            () -> m_altitude.AltitudeIsInAutoIntakePosition() &&
                m_extension.ExtensionIsInIntakePosition()),
        new ParallelCommandGroup(
            driveToGamePiece(m_drive),
            new InstantCommand(() -> m_intake.intakeCone())));
  }

  /**
   * could we do something like this?
   * Starting point
   * R1x = 4;
   * R1y = 0.35;
   * R1h = 5;
   * R2x = 4.5;
   * R2y = 0.3;
   * R3x = 5;
   * R3y = 0.35
   * R3h = 0;
   * B1x = 4;
   * B1y = -0.35;
   * B1h = -5;
   * B2x = 4.5;
   * B2y = -0.3;
   * B3x = 0;
   * B3y = -0.35;
   * B3h = 0;
   * 
   */
  public Command driveToGamePiece(DriveSubsystem m_drive) {
    // Create config for trajectory
    TrajectoryConfig config = AutoTrajectoryCommand.config(false);
    int sign = Autos.getAlliance() == Alliance.RED ? 1 : -1;

    // An example trajectory to follow. All units in meters.
    Trajectory pickupCargo = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing forward
        new Pose2d(5, sign * -0.1, Rotation2d.fromDegrees(sign * 5)),
        // NOTE: MUST have a waypoint. CANNOT be a straight line.
        List.of(new Translation2d(5.5, sign * -0.11)),
        // End 2 meters straight ahead of where we started still facing forward
        new Pose2d(6, sign * -0.1, Rotation2d.fromDegrees(sign * 0)),
        config);

    return AutoTrajectoryCommand.command(m_drive, pickupCargo);
  }
}