// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Henchman;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.Shared.Move.AutoMoveToPickupDirty;
import frc.robot.commands.Autos.Shared.Move.AutoPickupCubeDirty;
import frc.robot.commands.Autos.Shared.ScoreHigh.AutoScoreHighCone;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class AutoHenchman extends SequentialCommandGroup {
  public AutoHenchman(
      DriveSubsystem m_drive,
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    addCommands(
        // new ParallelCommandGroup(
        // Reset Heading & Odometry
        // new AutoResetOdometry(m_drive),
        // 1. Score Cone
        new AutoScoreHighCone(m_drive, m_altitude, m_extension, m_intake),
        // 2. Move to pickup cargo position
        new AutoMoveToPickupDirty(m_drive, m_altitude, m_extension),
        // 3. Intake down and pickup cargo
        new AutoPickupCubeDirty(m_drive, m_altitude, m_extension, m_intake),
        // 4. Intake up and move back to grid
        new AutoHenchmanReturnToScore(m_drive, m_altitude, m_extension, m_intake),
        // 5. Shoot cube
        new AutoHenchmanSecondScore(m_altitude, m_extension, m_intake),
        // 6. Go back to pickup again
        new AutoHenchmanReturnToPickup(m_drive, m_altitude, m_extension, m_intake));

  }

}