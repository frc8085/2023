// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDriveMetersAndTurn extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_drive;
    private double m_forwardMeters = 0;
    private double m_sidewaysMeters = 0;
    private double m_speed = 0;
    private double m_degrees = 0;
    private double m_rotationSpeed = 0;
    boolean forwardReached = false;
    boolean sidewaysReached = false;
    boolean turnComplete = false;

    public AutoDriveMetersAndTurn(DriveSubsystem drive, double forwardMeters, double sidewaysMeters, double speed,
            double degrees, double rotationSpeed) {
        m_drive = drive;
        m_forwardMeters = forwardMeters;
        m_sidewaysMeters = sidewaysMeters;
        m_speed = speed;
        m_degrees = degrees;
        m_rotationSpeed = rotationSpeed;
        addRequirements(m_drive);
    }

    // Reset the odomotry when the command is scheduled
    // Then run the drive command to travel backwards
    @Override
    public void initialize() {
        super.initialize();
        m_drive.resetOdometry(new Pose2d());
        m_drive.zeroHeading();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        super.execute();

        m_drive.drive(
                false,
                m_speed,
                forwardReached ? 0 : Math.signum(m_forwardMeters),
                sidewaysReached ? 0 : Math.signum(m_sidewaysMeters),
                turnComplete ? 0 : Math.signum(m_degrees) * -m_rotationSpeed,
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
        double currentForwardPose = m_drive.getPose().getX();
        double currentSidewaysPose = m_drive.getPose().getY();
        double currentDegrees = m_drive.getHeading();
        // Stop when the current position reaches
        // the desired backwards travel distance in meters

        if (m_forwardMeters > 0) {
            forwardReached = currentForwardPose >= m_forwardMeters;
        } else {
            forwardReached = currentForwardPose <= m_forwardMeters;
        }

        if (m_sidewaysMeters > 0) {
            sidewaysReached = currentSidewaysPose >= m_sidewaysMeters;
        } else {
            sidewaysReached = currentSidewaysPose <= m_sidewaysMeters;
        }

        if (m_degrees > 0) {
            turnComplete = currentDegrees >= m_degrees;
        } else {
            turnComplete = currentDegrees <= m_degrees;
        }

        return forwardReached && sidewaysReached && turnComplete;

    }

}
