// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeNoPID;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.Extension;

public class RunEjectCone extends SequentialCommandGroup {
    public RunEjectCone(
            Altitude m_altitude,
            Extension m_extension,
            IntakeNoPID m_intake) {
        addCommands(
                // 1. Prepare Drop Off Cone (lower altitude slightly)
                new PrepareDropOffCone(m_altitude),
                // 2. Run eject Cone
                new InstantCommand(() -> m_intake.ejectCone()),
                // 3. Wait .5 sec and then turn off intake
                new WaitCommand(.5)
                        .andThen(new InstantCommand(m_intake::stopIntake)),
                // 4. Return to Travel Position
                new PrepareTravel(m_extension, m_altitude));
    }
}