// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class DriveBackwardsToBalance extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_drive;
  private double m_speed = 0;
  private double m_slowSpeed = 0;
  private boolean reachedChargingStation = false;
  private boolean isBalanced = false;

  private double currentPitch;
  private boolean pitchTipped;

  public DriveBackwardsToBalance(DriveSubsystem drive, double speed, double slowSpeed) {
    m_drive = drive;
    // Take the magnitude of meters but ignore the sign
    // Just in case we provide a negative meters to this function by mistake
    m_speed = speed;
    m_slowSpeed = slowSpeed;
    addRequirements(m_drive);
  }

  // Reset the odomotry when the command is scheduled
  // Then run the drive command to travel backwards
  @Override
  public void initialize() {
    m_drive.resetOdometry(new Pose2d());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPitch = m_drive.getPitch();
    pitchTipped = reachedChargingStation && currentPitch < 0;

    if (currentPitch >= 10) {
      reachedChargingStation = true;
    }

    SmartDashboard.putBoolean("REACHED STATION", reachedChargingStation);
    SmartDashboard.putBoolean("BALACED", reachedChargingStation && isBalanced);

    m_drive.drive(
        false,
        reachedChargingStation ? pitchTipped ? m_slowSpeed / 2 : m_slowSpeed : m_speed,
        pitchTipped ? AutoConstants.kTravelForwards : AutoConstants.kTravelBackwards,
        0,
        0,
        true,
        false);
  }

  // Stop driving when the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
    new RunCommand(m_drive::lock, m_drive);
  }

  // End the command when we reach the desired pose in meters
  @Override
  public boolean isFinished() {
    isBalanced = currentPitch > 0 && currentPitch < 3;
    return reachedChargingStation && isBalanced;
  }
}
