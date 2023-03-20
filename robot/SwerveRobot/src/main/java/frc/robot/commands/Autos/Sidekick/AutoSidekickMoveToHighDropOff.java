package frc.robot.commands.Autos.Sidekick;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import frc.robot.Constants.AltitudeConstants;
import frc.robot.Constants.ExtensionConstants;
import frc.robot.commands.Extend;
import frc.robot.commands.RaiseLower;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.Extension;

// Move to High Drop Off position
public class AutoSidekickMoveToHighDropOff extends SequentialCommandGroup {
    public AutoSidekickMoveToHighDropOff(
            Altitude m_altitude,
            Extension m_extension) {
        addCommands(
                // 1. score
                new ParallelCommandGroup(
                        new Extend(m_extension, ExtensionConstants.kExtensionPositionHighDropOff),
                        new RaiseLower(m_altitude, AltitudeConstants.kAltitudeHighDropOffFinalPosition))

        //
        );

    }
}