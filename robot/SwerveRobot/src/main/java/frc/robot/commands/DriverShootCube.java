// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

public class DriverShootCube extends SequentialCommandGroup {
  public DriverShootCube(
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    addCommands(
        // Prepare Cube SHoot (move extension to proper position)
        new MoveToCubeShoot(m_extension),
        new WaitUntilCommand(() -> m_extension.ExtensionIsInCubeShootPosition()),
        // Run Eject Cube
        new ShootCube(m_altitude, m_extension, m_intake),
        // Return to Travel Position
        new MoveToTravelAfterScoring(m_extension, m_altitude));
  }
}
