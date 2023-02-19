// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robot.Constants.ExtensionConstants;
import static frc.robot.Constants.ElevatorConstants;

import java.time.Instant;

import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.Extension;

public class PrepareTravel extends SequentialCommandGroup {
        public PrepareTravel(
                        Extension m_Extension,
                        Altitude m_elevator) {
                addCommands(new ParallelCommandGroup(
                                new InstantCommand(() -> m_elevator.keepPosition(
                                                ElevatorConstants.kElevatorAltitudeTravelPosition)),
                                new InstantCommand(() -> m_Extension
                                                .keepPosition(ExtensionConstants.kExtensionPositionFullyRetracted))));

        }
}