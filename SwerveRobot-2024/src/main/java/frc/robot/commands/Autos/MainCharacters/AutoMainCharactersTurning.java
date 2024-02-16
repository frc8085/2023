// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.MainCharacters;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.Constants.OldAutoConstants;
import frc.robot.commands.AutoTurnToDegreeGyro;
import frc.robot.commands.Autos.Shared.Balance.AutoDecreasingSpeedBalance;
import frc.robot.commands.Autos.Shared.Balance.AutoConstantSpeedFinalBalance;
import frc.robot.commands.Autos.Shared.Balance.AutoPitchRollBalance;
import frc.robot.commands.Autos.Shared.Move.AutoDriveBackwardsMeters;
import frc.robot.commands.Autos.Shared.Move.AutoDriveForwardsMeters;
import frc.robot.commands.Autos.Shared.Move.AutoDriveSeconds;
import frc.robot.commands.Autos.Shared.Move.AutoDriveToReachStation;
import frc.robot.commands.Autos.Shared.Move.AutoTravelToFrontChargeStation;
import frc.robot.commands.Autos.Shared.ScoreHigh.AutoScoreHighCone;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

/**
 * Score High Cone
 * Backwards (meters)
 * Forwards (pitch)
 * Forwards (seconds)
 * Selected Balance Method
 */
public class AutoMainCharactersTurning extends SequentialCommandGroup {
  public AutoMainCharactersTurning(
      DriveSubsystem m_drive,
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake,
      Command balanceCommand) {
    addCommands(
        // 1. Score Cone
        new AutoScoreHighCone(m_drive, m_altitude, m_extension, m_intake),
        // 2. Move to leave community
        // Face intake up
        new InstantCommand(() -> System.out.println(Robot.getElapsedTime() + " AutoTurnToDegreeGyro")),
        new AutoTurnToDegreeGyro(180, m_drive, false),

        new InstantCommand(() -> System.out.println(Robot.getElapsedTime() + " AutoDriveForwardsMeters")),
        new AutoDriveForwardsMeters(m_drive, 4.75, 0.4),

        // Face intake down
        new InstantCommand(() -> System.out.println(Robot.getElapsedTime() + " AutoTurnToDegreeGyro")),
        new AutoTurnToDegreeGyro(0, m_drive, false),

        // 3. Move to charge station
        new InstantCommand(() -> System.out.println(Robot.getElapsedTime() + " AutoDriveToReachStation")),
        new AutoDriveToReachStation(
            m_drive,
            OldAutoConstants.kTravelForwards,
            OldAutoConstants.kDriveToStationSpeed,
            OldAutoConstants.kPitchReachedStationDegrees),

        new InstantCommand(() -> System.out.println(Robot.getElapsedTime() + " AutoDriveSeconds")),
        new AutoDriveSeconds(m_drive,
            OldAutoConstants.kTravelForwards,
            OldAutoConstants.kDriveToStationSpeed,
            OldAutoConstants.kChargingStationExtraDriveSeconds),

        new InstantCommand(() -> System.out.println(Robot.getElapsedTime() + " balanceCommand")),
        balanceCommand,

        new InstantCommand(() -> System.out.println(Robot.getElapsedTime() + " FinalLock")),
        new RunCommand(m_drive::lock, m_drive)

    );
  }

}