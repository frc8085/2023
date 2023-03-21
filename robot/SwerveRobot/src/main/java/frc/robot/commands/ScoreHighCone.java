// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.AltitudeConstants;
import frc.robot.Constants.ExtensionConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.Extension;

public class ScoreHighCone extends SequentialCommandGroup {
    public ScoreHighCone(
            Altitude m_altitude,
            Extension m_extension,
            Intake m_intake) {
        addCommands(
                // 1. Prepare Drop Off Cone (lower altitude slightly)
                new MoveToHighConeFinalDropOff(m_altitude),
                // Make sure the drop off cone altitude has been reached
                new WaitUntilCommand(() -> m_altitude.AltitudeIsInHighDropOffFinalPosition()),
                // Start Retracting at fixed speed until it reaches release position
                new InstantCommand(() -> m_extension.retractExtension())
                        .until(m_extension::ExtensionIsInReleasePosition),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> m_intake.ejectCone()),
                                new WaitCommand(IntakeConstants.kEjectWaitTime),
                                new InstantCommand(m_intake::stopIntake)),

                        // TODO: New test to run for Score High Cube
                        new SequentialCommandGroup(
                                new RaiseLower(m_altitude, AltitudeConstants.kAltitudeIntakePosition),
                                new WaitUntilCommand(() -> m_altitude.AltitudeIsInScoringPosition()),
                                new Extend(m_extension, ExtensionConstants.kExtensionPositionFullyRetracted),
                                new MoveToTravelAfterScoring(m_extension, m_altitude))

                )

        );
    }
}