// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeNoPID;

public class HoldCube extends SequentialCommandGroup {
    public HoldCube(
            IntakeNoPID m_intake) {
        addCommands(
                // 3. Run intake
                new InstantCommand(() -> m_intake.holdCube()));
    }
}