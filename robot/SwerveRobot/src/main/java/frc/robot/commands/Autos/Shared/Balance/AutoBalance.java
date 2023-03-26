// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Shared.Balance;

// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends CommandBase {
    private final DriveSubsystem m_drive;

    private static final double speedMetersPerSecond = 0.2;
    private static final double pitchToleranceDegress = 3.0;
    private static final double velocityToleranceDegreesPerSecond = 8.0;

    private double angleDegrees;

    public AutoBalance(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        angleDegrees = Double.POSITIVE_INFINITY;
    }

    @Override
    public void execute() {
        // Calculate charge station angle and velocity
        angleDegrees = m_drive.getRotation().getCos() * m_drive.getPitch().getDegrees()
                + m_drive.getRotation().getSin() * m_drive.getRoll().getDegrees();
        double angleVelocityDegreesPerSec = m_drive.getRotation().getCos()
                * m_drive.getPitchVelocity()
                + m_drive.getRotation().getSin() * m_drive.getRollVelocity();
        boolean shouldStop = (angleDegrees < 0.0 && angleVelocityDegreesPerSec > velocityToleranceDegreesPerSecond)
                || (angleDegrees > 0.0
                        && angleVelocityDegreesPerSec < -velocityToleranceDegreesPerSecond);

        // Send velocity to drive
        if (shouldStop) {
            m_drive.stop();
        } else {
            m_drive.drive(
                    speedMetersPerSecond,
                    (angleDegrees > 0.0 ? -1.0 : 1.0),
                    0,
                    0,
                    true,
                    false);
        }

    }

    @Override
    public void end(boolean interrupted) {
        m_drive.lock();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(angleDegrees) < pitchToleranceDegress;
    }
}