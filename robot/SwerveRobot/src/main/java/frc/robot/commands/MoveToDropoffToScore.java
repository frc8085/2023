// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static frc.robot.Constants.ExtensionConstants;

import frc.robot.subsystems.Extension;

public class MoveToDropoffToScore extends SequentialCommandGroup {
    public MoveToDropoffToScore(
            Extension m_extension) {
        addCommands(new InstantCommand(
                () -> m_extension.keepPositionInches(ExtensionConstants.kExtensionPositionInchesFullyRetracted)),
                new WaitUntilCommand(() -> m_extension.ExtensionIsInDropOffReturnPosition()));

    }
}
