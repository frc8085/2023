// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoRotateDegrees extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_drive;
    private double m_degrees;

    public AutoRotateDegrees(DriveSubsystem drive, double degrees) {
        m_drive = drive;
        m_degrees = degrees;
        addRequirements(m_drive);
    }

    // Reset the odomotry when the command is scheduled
    // Then run the drive command to travel backwards
    @Override
    public void initialize() {
        super.initialize();

        m_drive.zeroHeading();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        super.execute();

        m_drive.drive(
                false,
                0.3,
                // AutoConstants.kMaxSpeedMetersPerSecond,
                0,
                0,
                -0.3,
                true,
                false);
    }

    // Stop driving when the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    // End the command when we reach the desired pose in meters
    @Override
    public boolean isFinished() {
        double currentDegrees = m_drive.getHeading();

        boolean turnComplete;
        if (m_degrees > 0) {
            turnComplete = currentDegrees >= m_degrees;
        } else {
            turnComplete = currentDegrees <= m_degrees;
        }

        return turnComplete;
    }
}
