// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.Extension;

public class IntakeCargo extends SequentialCommandGroup {
    public IntakeCargo(
            Altitude m_altitude,
            Extension m_extension,
            Intake m_intake) {
        addCommands(
                // 1. Prepare intake
                new MoveToIntake(m_extension, m_altitude),
                new WaitUntilCommand(
                        () -> m_altitude.AltitudeIsInIntakePosition() &&
                                m_extension.ExtensionIsInIntakePosition()),
                // 2. Run intake
                new InstantCommand(() -> m_intake.intakeCone()));
    }
}