// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.MainCharacters;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OldAutoConstants;
import frc.robot.commands.Autos.Shared.Balance.AutoFinalBalanceWithTimeout;
import frc.robot.commands.Autos.Shared.Move.AutoDriveBackwardsMeters;
import frc.robot.commands.Autos.Shared.Move.AutoDriveSeconds;
import frc.robot.commands.Autos.Shared.Move.AutoDriveToReachStation;
import frc.robot.commands.Autos.Shared.Move.AutoTravelToFrontChargeStation;
import frc.robot.commands.Autos.Shared.ScoreHigh.AutoScoreHighCone;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

/**
 * Michael
 *
 * Score
 * Backwards (meters)
 * Forwards (pitch)
 * Forwards (seconds)
 * Final Balance (with timeout)
 */
public class AutoMichael extends SequentialCommandGroup {
  public AutoMichael(
      DriveSubsystem m_drive,
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    addCommands(
        // 1. Score Cone
        new AutoScoreHighCone(m_drive, m_altitude, m_extension, m_intake),
        // 2. Move to leave community
        new AutoDriveBackwardsMeters(m_drive, 4.75, 0.4),
        new WaitCommand(1),
        // 3. Move to charge station
        new AutoDriveToReachStation(
            m_drive,
            OldAutoConstants.kTravelForwards,
            OldAutoConstants.kDriveToStationSpeed,
            10),

        new AutoDriveSeconds(m_drive,
            OldAutoConstants.kTravelForwards,
            OldAutoConstants.kDriveToStationSpeed,
            1),

        new AutoFinalBalanceWithTimeout(m_drive),
        new RunCommand(m_drive::lock, m_drive)

    );
  }

}