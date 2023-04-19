// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Shared.Move;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveSeconds extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_drive;
  private double m_seconds;
  private double m_speed;
  private double m_direction;

  private Timer timer = new Timer();

  public AutoDriveSeconds(DriveSubsystem drive, double direction, double speed, double seconds) {
    m_drive = drive;
    m_direction = direction;
    m_speed = speed;
    m_seconds = seconds;

    addRequirements(m_drive);
  }

  // Reset the odomotry when the command is scheduled
  // Then run the drive comma
  public void initialize() {
    super.initialize();

    timer.reset();
    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(
        m_speed,
        m_direction,
        0,
        0,
        true,
        false);
  }

  // Stop driving when the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // End the command when we've driven desired time
  @Override
  public boolean isFinished() {
    return timer.get() >= m_seconds;
  }
}