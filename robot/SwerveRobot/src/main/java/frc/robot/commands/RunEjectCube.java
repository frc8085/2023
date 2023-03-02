// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class RunEjectCube extends SequentialCommandGroup {
  public RunEjectCube(
      Intake m_intake) {
    addCommands(
        new InstantCommand(() -> m_intake.ejectCube()),
        // 3. Wait X sec and then turn off intake
        new WaitCommand(IntakeConstants.kEjectWaitTime)
            .andThen(new InstantCommand(m_intake::stopIntake)));
  }
}