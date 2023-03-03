// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.Constants.ExtensionConstants;

import frc.robot.subsystems.Extension;

public class PrepareMidCubeDropOff extends SequentialCommandGroup {
    public PrepareMidCubeDropOff(
            Extension m_extension) {
        addCommands(
                new InstantCommand(() -> m_extension
                        .keepPosition(ExtensionConstants.kExtensionPositionCubeShooter)));

    }
}
