// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  public static CommandBase autoMid(DriveSubsystem m_drive) {
    return Commands.sequence(
        new DriveBackwardsToBalance(m_drive, .25, .15));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
