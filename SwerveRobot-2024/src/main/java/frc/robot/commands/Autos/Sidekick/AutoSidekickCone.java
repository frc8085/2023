// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Sidekick;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.Shared.Move.AutoTravelToPickupAndStartLoweringIntakeClean;
import frc.robot.commands.Autos.Shared.Move.AutoTravelToPickupClean;
import frc.robot.commands.Autos.Shared.Move.AutoTravelAndPickupConeClean;
import frc.robot.commands.Autos.Shared.Move.AutoTravelAndPickupCubeClean;
import frc.robot.commands.Autos.Shared.ScoreHigh.AutoScoreHighCone;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class AutoSidekickCone extends SequentialCommandGroup {
  public AutoSidekickCone(
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
        new AutoTravelToPickupClean(m_drive, m_altitude, m_extension),
        // 3. Intake down and pickup cargo
        new AutoTravelAndPickupConeClean(m_drive, m_altitude, m_extension, m_intake),
        // 4. Intake up and move back to grid
        new AutoSidekickReturnToScore(m_drive, m_altitude, m_extension, m_intake),
        // 5. Shoot cube
        new AutoSidekickSecondScoreCone(m_altitude, m_extension, m_intake)
    // 6. Go back to pickup again
    // new AutoSidekickReturnToPickup(m_drive, m_altitude, m_extension, m_intake)
    );

  }

}