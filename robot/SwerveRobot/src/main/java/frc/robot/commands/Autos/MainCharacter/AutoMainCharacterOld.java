// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.MainCharacter;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Autos.Shared.Balance.AutoMainCharacterBalance;
import frc.robot.commands.Autos.Shared.Move.AutoDriveBackwardsMeters;
import frc.robot.commands.Autos.Shared.Move.AutoDriveForwardsMeters;
import frc.robot.commands.Autos.Shared.ScoreHigh.AutoScoreHighCone;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

/** Original Main Character - Does not use Swerve Spline Trajectories. */
public class AutoMainCharacterOld extends SequentialCommandGroup {
  public AutoMainCharacterOld(
      DriveSubsystem m_drive,
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    addCommands(
        // 1. Score Cone
        new AutoScoreHighCone(m_drive, m_altitude, m_extension, m_intake),
        // 2. Move to leave community
        new AutoDriveBackwardsMeters(m_drive, 4.75, 0.4),
        // 3. Let the charging station stabilize
        new WaitCommand(1),
        // 4. Drive onto the charging station
        new AutoDriveForwardsMeters(m_drive, 2.50),
        // 5. Attempt to balance until time runs out
        new AutoMainCharacterBalance(m_drive),
        // 6. We should never reach this command, but adding as fail-safe
        new RunCommand(m_drive::lock, m_drive));
  }

}