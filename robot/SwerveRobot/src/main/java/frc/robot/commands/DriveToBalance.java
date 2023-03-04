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

public class DriveToBalance extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_drive;
  private double m_speed = 0;
  private boolean isBalanced = false;

  public DriveToBalance(DriveSubsystem drive, double speed) {
    m_drive = drive;
    // Take the magnitude of meters but ignore the sign
    // Just in case we provide a negative meters to this function by mistake
    m_speed = speed;
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
    double currentPitch = m_drive.getPitch();
    isBalanced = currentPitch > 14;

    m_drive.drive(
        false,
        m_speed,
        AutoConstants.kTravelBackwards,
        0,
        0,
        true,
        false);

    SmartDashboard.putBoolean("BALACED", isBalanced);

  }

  // Stop driving when the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
    new RunCommand(m_drive::lock, m_drive);
  }

  // End the command when we reach the desired pose in meters
  @Override
  public boolean isFinished() {
    return isBalanced;
  }
}
