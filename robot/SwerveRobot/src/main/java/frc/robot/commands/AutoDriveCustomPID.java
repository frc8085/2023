// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDriveCustomPID extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final boolean TUNING_MODE = true;

  private final DriveSubsystem m_drive;
  private boolean isBalanced = false;

  private double setpointMetersPerSecond = 0.1; // Desired speed in m/s
  private double kP = 8; // Proportional gain

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
    m_drive.resetOdometry(new Pose2d());
    m_drive.zeroHeading();

    if (TUNING_MODE) {
      SmartDashboard.putNumber("CustomBalance P Gain", kP);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();

    currentPitch = m_drive.getPitch();

    double currentSpeedMetersPerSecond = m_drive.getCurrentVelocity();
    double error = setpointMetersPerSecond - currentSpeedMetersPerSecond;
    double outputSpeed = Math.min(error * kP, 4.8);
    double scaledSpeed = outputSpeed / 4.8;

    m_drive.drive(
        false,
        scaledSpeed, // error * kP
        // Math.signum(-currentPitch),
        -1,
        0, 0, true, false);

    log();
  }

  private void log() {
    SmartDashboard.putBoolean("FINAL BALANCED", isBalanced);

    if (TUNING_MODE) {
      SmartDashboard.putNumberArray("Pitch readings", readings);
      SmartDashboard.putNumber("Pitch: Current", currentPitch);
      SmartDashboard.putNumber("Pitch: Average", averageReading);

      // Read proportional gain from
      double p = SmartDashboard.getNumber("CustomBalance P Gain", kP);

      // If PID coefficients on SmartDashboard have changed, write new values to
      // controller
      if ((p != kP)) {
        kP = p;
      }
    }
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

    // TEST 1 - drive 2 meters
    boolean traveledTwoMeters = Math.abs(m_drive.getPose().getX()) >= 2;

    // TEST 2 - drive until 0 degrees (instantaneous measurement)
    boolean instantaneousBalance = Math.abs(currentPitch) <= 1;

    return traveledTwoMeters;
  }
}
