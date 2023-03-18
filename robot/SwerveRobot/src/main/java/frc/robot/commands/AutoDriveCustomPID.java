// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import java.util.Arrays;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDriveCustomPID extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_drive;
  private boolean isBalanced = false;

  private double setpointMetersPerSecond = 0.1; // Desired speed in m/s
  private double kP = 0.1; // Proportional gain

  private int readingIndex = 0;
  private int windowSize = 20;
  private double[] readings = new double[windowSize];
  private double averageReading;
  private double impossibleReading = 99999;

  double currentPitch;

  public AutoDriveCustomPID(DriveSubsystem drive) {
    m_drive = drive;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();

    currentPitch = m_drive.getPitch();

    double currentSpeedMetersPerSecond = m_drive.getCurrentVelocity();
    double error = setpointMetersPerSecond - currentSpeedMetersPerSecond;
    double outputSpeed = error * kP;

    m_drive.drive(
        false,
        outputSpeed, // error * kP
        Math.signum(-currentPitch),
        0, 0, true, false);

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
    // Store a reading for each slot in our window size
    readings[readingIndex % windowSize] = currentPitch;
    readingIndex++;

    // Average the readings over the window size
    averageReading = readingIndex >= windowSize
        ? Arrays.stream(readings).sum() / windowSize
        : impossibleReading;

    isBalanced = Math.abs(averageReading) < 1;

    return isBalanced;
  }
}
