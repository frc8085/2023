// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.HailMary;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ScoreCube;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class AutoHailMarySecondScore extends SequentialCommandGroup {
  public AutoHailMarySecondScore(
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    addCommands(
        new ScoreCube(m_altitude, m_extension, m_intake));

  }

}