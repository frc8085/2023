// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class AutoFinalBalance extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_drive;
    private double m_speed = AutoConstants.kFinalBalanceSpeed;
    private boolean isBalanced = false;

    public AutoFinalBalance(DriveSubsystem drive) {
        m_drive = drive;
        // Take the magnitude of meters but ignore the sign
        // Just in case we provide a negative meters to this function by mistake
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currentPitch = m_drive.getPitch();
        isBalanced = currentPitch >= 0.5 && currentPitch <= 1.5;

        m_drive.drive(
                false,
                m_speed,
                AutoConstants.kTravelBackwards,
                0,
                0,
                true,
                false);

        SmartDashboard.putBoolean("FINAL BALANCED", isBalanced);
    }

    // Stop driving when the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    // End the command when we reach the desired pose in meters
    @Override
    public boolean isFinished() {
        return isBalanced;
    }
}
