// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Shared;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

/**
 * Aim using the limelight degree rotation to target
 */
public class AutoPositionWithLimelight extends CommandBase {
  private final DriveSubsystem m_drive;
  private final Limelight m_limelight;

  // Creates a MedianFilter with a window size of 5 samples
  MedianFilter filter = new MedianFilter(5);
  MedianFilter setpointFilter = new MedianFilter(5);

  double medianReading;

  public AutoPositionWithLimelight(DriveSubsystem drive, Limelight limelight) {
    // Require the drive and limelight
    m_drive = drive;
    m_limelight = limelight;
    addRequirements(m_drive, m_limelight);
  }

  @Override
  public void execute() {
    super.execute();
    double driveSetpoint = m_limelight.getXDistanceFromTarget();

    System.out.println("X from target" + driveSetpoint);
    // Use the median from the last 5 readings
    // We do this because the input can be erratic
    // Median is more robust than average
    medianReading = filter.calculate(driveSetpoint);
    double direction = Math.signum(medianReading);
    m_drive.drive(
        0.1,
        0,
        direction,
        0,
        true,
        false);
    ;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    // Get everything in a safe starting state.
    super.initialize();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    boolean targetVisible = m_limelight.getIsTargetFound();
    boolean withinTolerance = Math.abs(medianReading) <= 2.5;
    // End this Command if we reached our setpoint OR we don't have a target visible
    return !targetVisible || withinTolerance;
  }
}
