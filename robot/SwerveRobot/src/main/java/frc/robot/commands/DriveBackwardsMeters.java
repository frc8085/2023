// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveBackwardsMeters extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_drive;
  private double m_meters = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveBackwardsMeters(DriveSubsystem drive, double meters) {
    m_drive = drive;
    m_meters = Math.abs(meters);
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_drive.zeroHeading();
    m_drive.drive(
        false,
        0.1,
        -1,
        0,
        0,
        true,
        false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentPose = m_drive.getPose().getX();
    return currentPose <= -m_meters;
  }
}
