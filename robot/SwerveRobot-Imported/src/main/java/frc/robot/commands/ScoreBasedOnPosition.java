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

public class ScoreBasedOnPosition extends SequentialCommandGroup {
  public ScoreBasedOnPosition(
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {

    // Position 1: Travel Position - Run Score Cube
    // Position 2: High Cone Position (Scoring Altitude, High Drop Off Extension) -
    // Run Score High Cone
    // Position 3: Mid Cone Position (Scoring Altitude, Mid Drop Off Extension) -
    // Run Score Mid Cone

    addCommands(
        // If Altitude is in Scoring or Travel Position, Score
        m_altitude.AltitudeIsInScoringPosition() || m_altitude.AltitudeIsInTravelPosition()
            ? new ConditionalCommand(
                new ScoreCube(m_altitude, m_extension, m_intake),
                // If Altitude is in scoring position, lower altitude slightly, then drop off
                // cone and then return to travel simultaneously
                new ConditionalCommand(
                    new ScoreHighCone(m_altitude, m_extension, m_intake),
                    // If Extension is in mid position
                    new ScoreMidCone(m_altitude, m_extension, m_intake),
                    () -> m_extension.ExtensionIsInHighScoringPosition()),
                // Is Altitude in travel position?
                () -> m_altitude.AltitudeIsInTravelPosition())
            // If Altitude is not, do nothing
            : new InstantCommand());
  }
}