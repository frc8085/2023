// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IntakeNoPID;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.Extension;

public class RunIntakeCube extends SequentialCommandGroup {
    public RunIntakeCube(
            Altitude m_altitude,
            Extension m_extension,
            IntakeNoPID m_intake) {
        addCommands(
                // 1. Prepare intake
                new PrepareIntake(m_extension, m_altitude),
                // 2. Wait until at setpoint (altitude and extension at intake position)
                new WaitUntilCommand(() -> m_altitude.AltitudeIsInIntakePosition()),
                // TODO: Write function in Extension subsystem that
                // returns if its at intake position
                // new WaitUntilCommand(() -> m_extension.ExtensionIsInIntakePosition()),

                // 3. Run intake
                new InstantCommand(() -> m_intake.intakeCube()));
    }
}
