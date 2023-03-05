//Copyight (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.Extension;

public class ScoreCube extends SequentialCommandGroup {
  public ScoreCube(
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    addCommands(
        new InstantCommand(() -> m_intake.ejectCube()),
        // 3. Wait X sec and then turn off intake
        new WaitCommand(IntakeConstants.kEjectWaitTime),
        new InstantCommand(m_intake::stopIntake));

  }
}