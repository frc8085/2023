// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AltitudeConstants;
import frc.robot.subsystems.Altitude;

/**
 * public class PrepareDropOffCone extends SequentialCommandGroup {
 * public PrepareDropOffCone(Altitude m_altitude) {
 * new InstantCommand(() ->
 * m_altitude.keepPosition(AltitudeConstants.kAltitudeDropOffFinalPosition));
 * }
 * }
 */

// TODO: Convert this to a lower altitude by position not by time

public class PrepareDropOffCone extends SequentialCommandGroup {
    public PrepareDropOffCone(Altitude m_altitude) {
        addCommands(
                new InstantCommand(() -> m_altitude.lowerAltitude()),
                new WaitCommand(.5)
                        .andThen(new InstantCommand(() -> m_altitude.stopAltitude())));
    }
}
