// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Shared.ScoreHigh;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ExtensionConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Extend;
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
                                new InstantCommand(() -> m_extension.retractExtension())
                                                .until(m_extension::ExtensionIsInReleasePosition),
                                new InstantCommand(() -> m_intake.ejectCone()),
                                new WaitCommand(IntakeConstants.kAutoEjectWaitTime),
                                new InstantCommand(m_intake::stopIntake));
        }
}
