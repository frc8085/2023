// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Extension;

public class SetStartingExtensionPosition extends SequentialCommandGroup {
    public SetStartingExtensionPosition(
            Extension m_extension) {
        addCommands(new ParallelCommandGroup(
                new InstantCommand(m_extension::moveToStartingPosition, m_extension)));
    }
}
