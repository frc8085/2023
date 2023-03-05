// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.Constants.ExtensionConstants;

import frc.robot.Constants.AltitudeConstants;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.Extension;

public class PrepareHighCubeDropOff extends SequentialCommandGroup {
  public PrepareHighCubeDropOff(
      Extension m_extension, Altitude m_altitude) {
    addCommands(
        new InstantCommand(() -> m_extension
            .keepPosition(ExtensionConstants.kExtensionPositionHighCubeShooter)),
        new InstantCommand(() -> m_altitude
            .keepPosition(AltitudeConstants.kAltitudeHighCubeShootPosition)));

  }
}
