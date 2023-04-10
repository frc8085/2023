// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Henchman;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DropCube;
import frc.robot.commands.ScoreHighCube;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class AutoHenchmanSecondScoreCone extends SequentialCommandGroup {
  public AutoHenchmanSecondScoreCone(
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    addCommands(
        new DropCube(m_altitude, m_extension, m_intake));

  }

}