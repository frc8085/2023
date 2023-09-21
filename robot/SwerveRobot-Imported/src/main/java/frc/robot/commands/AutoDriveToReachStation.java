// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDriveToReachStation extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_drive;
    private double m_speed = AutoConstants.kDriveToStationSpeed;
    private boolean reachedChargingStation = false;

    public AutoDriveToReachStation(DriveSubsystem drive) {
        m_drive = drive;
        // Take the magnitude of meters but ignore the sign
        // Just in case we provide a negative meters to this function by mistake
        addRequirements(m_drive);
    }

    // Reset the odomotry when the command is scheduled
    // Then run the drive command to travel backwards
    @Override
    public void initialize() {
        super.initialize();
        m_drive.resetOdometry(new Pose2d());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        super.execute();

        double currentPitch = m_drive.getPitch();

        if (!reachedChargingStation && currentPitch >= 10) {
            reachedChargingStation = true;
        }

        m_drive.drive(
                false,
                m_speed,
                AutoConstants.kTravelBackwards,
                0,
                0,
                true,
                false);

        SmartDashboard.putBoolean("REACHED STATION", reachedChargingStation);
        m_drive.logSwerveStates();
    }

    // Stop driving when the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    // End the command when we reach the desired pose in meters
    @Override
    public boolean isFinished() {
        return reachedChargingStation;
    }
}
