// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Shared.Balance;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Autos.Autos;
import frc.robot.commands.Autos.Autos.Alliance;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;

/**
 * Aim using the limelight degree rotation to target
 */
public class AprilTagBalance extends CommandBase {
  private final DriveSubsystem m_drive;
  private final LimelightSubsystem m_limelight;
  private final Alliance alliance;
  private double poseX;
  private double setpoint = 0;
  private double tolerance = 0.15;

  // Creates a MedianFilter with a window size of 5 samples
  MedianFilter filter = new MedianFilter(5);
  MedianFilter setpointFilter = new MedianFilter(5);

  double medianReading;

  public AprilTagBalance(DriveSubsystem drive, LimelightSubsystem limelight) {
    // Require the drive and limelight
    m_drive = drive;
    m_limelight = limelight;
    alliance = Autos.getAlliance();
    addRequirements(m_drive, m_limelight);
  }

  @Override
  public void execute() {
    super.execute();
    double direction = 0;

    if (m_limelight.hasTargetRight()) {
      if (alliance == Alliance.RED) {
        poseX = m_limelight.getBotPoseRed().getX();
      } else {
        poseX = m_limelight.getBotPoseBlue().getX();
      }

      // Use the median from the last 5 readings
      // We do this because the input can be erratic
      // Median is more robust than average
      medianReading = filter.calculate(poseX);
      direction = Math.signum(medianReading - setpoint);

      System.out.println("X meters from apriltag: " + medianReading);
      System.out.println("X travel direction: " + direction);

      m_drive.drive(
          0.1,

          -direction,
          0,
          0,
          true,
          false);
    } else {
      m_drive.lock();
    }
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
    boolean withinTolerance = Math.abs(medianReading - setpoint) <= tolerance;
    // End this Command if we reached our setpoint OR we don't have a target visible
    return withinTolerance;
  }
}
