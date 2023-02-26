//Copyight (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeNoPID;
import frc.robot.Constants.IntakeNoPIDConstants;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.Extension;

public class RunEjectBasedOnAltitude extends SequentialCommandGroup {
        public RunEjectBasedOnAltitude(
                        Altitude m_altitude,
                        Extension m_extension,
                        IntakeNoPID m_intake) {
                addCommands(new ConditionalCommand(
                                // If Altitude is in travel position, eject the object at full speed
                                new SequentialCommandGroup(
                                                new InstantCommand(() -> m_intake.ejectCube()),
                                                // 3. Wait X sec and then turn off intake
                                                new WaitCommand(IntakeNoPIDConstants.kEjectWaitTime)
                                                                .andThen(new InstantCommand(m_intake::stopIntake))),
                                // If Altitude is in scoring position, lower altitude slightly, then drop off
                                // cone and then return to travel simultaneously
                                new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                                // 1. Prepare Drop Off Cone (lower altitude slightly)
                                                                new PrepareDropOffCone(m_altitude),
                                                                new ParallelCommandGroup(
                                                                                new SequentialCommandGroup(
                                                                                                // 2. Run eject Cone
                                                                                                new InstantCommand(
                                                                                                                () -> m_intake
                                                                                                                                .ejectCone()),
                                                                                                // 3. Wait X sec and
                                                                                                // then turn off
                                                                                                // intake
                                                                                                new WaitCommand(IntakeNoPIDConstants.kEjectWaitTime)
                                                                                                                .andThen(new InstantCommand(
                                                                                                                                m_intake::stopIntake))),
                                                                                // 4. Return to Travel Position
                                                                                new PrepareTravelAfterScoring(
                                                                                                m_extension,
                                                                                                m_altitude))),
                                                // If Altitude is not in travel or scoring position, then just eject
                                                // cone
                                                new InstantCommand(() -> m_intake.ejectCone()),
                                                () -> m_altitude.AltitudeIsInScoringPosition()),
                                // Is Altitude in travel position?
                                () -> m_altitude.AltitudeIsInTravelPosition()));
        }
}