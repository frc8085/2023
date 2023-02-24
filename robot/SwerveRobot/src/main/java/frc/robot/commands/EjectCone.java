// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.Constants.ExtensionConstants;
import static frc.robot.Constants.AltitudeConstants;

import java.time.Instant;

import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

public class EjectCone extends SequentialCommandGroup {
    public EjectCone(
            Intake m_intake,
            Extension m_extension,
            Altitude m_altitude) {
        addCommands(new ParallelCommandGroup(
                new InstantCommand(() -> m_intake.keepPosition(
                        AltitudeConstants.kAltitudeTravelPosition)),
                new WaitCommand(.25)
                        .andThen(new InstantCommand(() -> m_extension
                                .keepPosition(ExtensionConstants.kExtensionPositionFullyRetracted)))));

    }
}
