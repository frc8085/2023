// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Shared.ScoreHigh;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.MoveToTravelAfterScoring;
import frc.robot.commands.Autos.Shared.AutoMoveToTravelAfterScoring;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class AutoEjectCone extends SequentialCommandGroup {
  public AutoEjectCone(
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    addCommands(
        // Start Retracting at fixed speed until it reaches release position
        new InstantCommand(() -> m_extension.retractExtension())
            .until(m_extension::ExtensionIsInReleasePosition),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> m_intake.ejectCone()),
                new WaitCommand(IntakeConstants.kEjectWaitTime),
                new InstantCommand(m_intake::stopIntake)),
            new AutoMoveToTravelAfterScoring(m_extension, m_altitude))

    );
  }
}
