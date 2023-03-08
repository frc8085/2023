// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AltitudeConstants;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public final class Autos {
    public static CommandBase balance(DriveSubsystem m_drive) {
        return Commands.sequence(
                new AutoDriveToReachStation(m_drive),
                new AutoDriveToBalance(m_drive),
                new AutoFinalBalance(m_drive),
                new RunCommand(m_drive::lock, m_drive));
    }

    public static CommandBase scoreHigh(DriveSubsystem m_drive, Altitude m_altitude,
            Extension m_extension,
            Intake m_intake) {
        return Commands.sequence(
                new MoveToHighConeDropOff(m_extension, m_altitude)
                        .until(() -> m_extension.ExtensionIsInHighScoringPosition()),
                new WaitUntilCommand(
                        () -> m_extension.ExtensionIsInHighScoringPosition()),
                new InstantCommand(
                        () -> m_altitude.keepPosition(
                                AltitudeConstants.kAltitudeHighDropOffPosition)),
                new WaitCommand(.5),
                new ScoreHighCone(m_altitude, m_extension, m_intake));
    }

    public static CommandBase scoreHighLeaveAndPickup(DriveSubsystem m_drive, Altitude m_altitude,
            Extension m_extension,
            Intake m_intake) {
        return Commands.sequence(
                scoreHigh(m_drive, m_altitude, m_extension, m_intake),
                new AutoDriveBackwardsMeters(m_drive, 3.6),
                new InstantCommand(m_drive::stop));
    }

    public static CommandBase scoreHighAndBalance(DriveSubsystem m_drive, Altitude m_altitude,
            Extension m_extension,
            Intake m_intake) {
        return Commands.sequence(
                scoreHigh(m_drive, m_altitude, m_extension, m_intake),
                balance(m_drive));
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
