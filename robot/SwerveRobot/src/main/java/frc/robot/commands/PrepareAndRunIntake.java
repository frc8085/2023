// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.IntakeNoPID;

public class PrepareAndRunIntake extends SequentialCommandGroup {
    public PrepareAndRunIntake(
            Extension m_extension,
            Altitude m_altitude,
            IntakeNoPID m_intake) {
        addCommands(
                new PrepareIntake(m_extension, m_altitude),
                new InstantCommand(() -> m_intake.intakeCone()));
    }
}