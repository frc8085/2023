// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.Extension;

public class RunIntakeConeFromShelf extends SequentialCommandGroup {
  public RunIntakeConeFromShelf(
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    addCommands(
        // 1. Prepare Shelf Pickup
        new PrepareDoubleSubstationPickup(m_extension, m_altitude),
        // Run intakeCone from Shelf
        new InstantCommand(() -> m_intake.intakeCone()));
  }
}
