// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

public final class Autos {
  public static CommandBase balance(DriveSubsystem m_drive) {
    return Commands.sequence(
        new DriveToReachStation(m_drive),
        new DriveToBalance(m_drive),
        new FinalBalance(m_drive),
        new RunCommand(m_drive::lock, m_drive));
  }

  public static CommandBase scoreHighAndBalance(DriveSubsystem m_drive, Altitude m_altitude, Extension m_extension,
      Intake m_intake) {
    return Commands.sequence(
        new ScoreHighCube(m_altitude, m_extension, m_intake),
        balance(m_drive));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
