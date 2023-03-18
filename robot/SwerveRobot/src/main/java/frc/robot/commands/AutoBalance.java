// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class AutoBalance extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_drive;
  private double currentPitch;
  private int iteration = 1;
  private int scalingFactor = 100;
  private double startingSpeed = .4;

  public AutoBalance(DriveSubsystem drive) {
    m_drive = drive;
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
    iteration++;
    currentPitch = m_drive.getPitch();

    boolean isBalanced = Math.abs(currentPitch) <= 2;

    double speed = startingSpeed / (1 + iteration / scalingFactor);
    System.out.println("CURRENT SPEED: " + speed);

    if (isBalanced) {
      m_drive.lock();
    } else {
      m_drive.drive(
          false,
          speed,
          Math.signum(-currentPitch),
          0,
          0,
          true,
          false);
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

    return false;
  }
}
