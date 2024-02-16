// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Shared.Move;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveToReachStation extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_drive;
  private double m_speed;
  private double m_pitchSetpoint;
  private double m_direction;
  private boolean reachedChargingStation = false;

  public AutoDriveToReachStation(DriveSubsystem drive, double direction, double speed, double pitchSetpoint) {
    m_drive = drive;
    m_direction = direction;
    m_speed = speed;
    m_pitchSetpoint = pitchSetpoint;
    // Take the magnitude of meters but ignore the sign
    // Just in case we provide a negative meters to this function by mistake
    addRequirements(m_drive);
  }

  // Reset the odomotry when the command is scheduled
  // Then run the drive command to travel backwards
  @Override
  public void initialize() {
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();

    double currentPitch = m_drive.getPitch().getDegrees();

    boolean atSetpoint = Math.abs(currentPitch) >= m_pitchSetpoint;

    if (!reachedChargingStation && atSetpoint) {
      reachedChargingStation = true;
    }

    m_drive.drive(
        m_speed,
        m_direction,
        0,
        0,
        false,
        false);
  }

  // Stop driving when the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // End the command when we reach the desired pose in meters
  @Override
  public boolean isFinished() {
    return reachedChargingStation;
  }
}