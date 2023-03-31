// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.MainCharacter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.Shared.Move.AutoTravelToFrontChargeStation;
import frc.robot.commands.Autos.Shared.ScoreHigh.AutoScoreHighCone;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class AutoMainCharacter extends SequentialCommandGroup {
  public AutoMainCharacter(
      DriveSubsystem m_drive,
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    addCommands(
        // 1. Score Cone
        new AutoScoreHighCone(m_drive, m_altitude, m_extension, m_intake),
        // 2. Move to leave community
        new AutoMainCharacterLeaveCommunity(m_drive, m_altitude, m_extension),
        // 3. Move to charge station
        new AutoMainCharacterPickupCubeClean(m_drive, m_altitude, m_extension, m_intake),
        new AutoMainCharacterReturnToScore(m_drive, m_altitude, m_extension, m_intake)
    // 5. Shoot cube
    // new AutoBalance(m_altitude, m_extension, m_intake)

    );
  }

}