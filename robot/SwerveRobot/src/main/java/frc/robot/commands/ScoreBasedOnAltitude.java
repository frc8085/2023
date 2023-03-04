//Copyight (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.Extension;

public class ScoreBasedOnAltitude extends SequentialCommandGroup {
  public ScoreBasedOnAltitude(
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    addCommands(new ConditionalCommand(
        new ScoreCube(m_altitude, m_extension, m_intake),
        // If Altitude is in scoring position, lower altitude slightly, then drop off
        // cone and then return to travel simultaneously
        new ConditionalCommand(
            new ScoreCone(m_altitude, m_extension, m_intake),
            // If Altitude is not in travel or scoring position, then just eject
            // cone
            new InstantCommand(() -> m_intake.ejectCone()),
            () -> m_altitude.AltitudeIsInScoringPosition()),
        // Is Altitude in travel position?
        () -> m_altitude.AltitudeIsInTravelPosition()));
  }
}