// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static frc.robot.Constants.ExtensionConstants;
import static frc.robot.Constants.AltitudeConstants;

import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.Extension;

public class PrepareDoubleSubstationPickup extends SequentialCommandGroup {
  public PrepareDoubleSubstationPickup(
      Extension m_extension,
      Altitude m_altitude) {
    addCommands(new ParallelCommandGroup(
        new InstantCommand(() -> m_extension
            .keepPosition(ExtensionConstants.kExtensionPositionMidDropOff)),
        new WaitUntilCommand(() -> m_extension.ExtensionIsInMidScoringPosition()),
        new InstantCommand(() -> m_altitude.keepPosition(
            AltitudeConstants.kAltitudeDoubleSubstationPosition))));
  }
}