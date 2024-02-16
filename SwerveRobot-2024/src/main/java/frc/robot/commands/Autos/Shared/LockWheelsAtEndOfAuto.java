// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Shared;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

/** Reset the heading and odometry */
public class LockWheelsAtEndOfAuto extends SequentialCommandGroup {
  public LockWheelsAtEndOfAuto(
      DriveSubsystem m_drive) {
    addCommands(
        new WaitCommand(14.9),
        new RunCommand(m_drive::lock, m_drive));
  }
}
