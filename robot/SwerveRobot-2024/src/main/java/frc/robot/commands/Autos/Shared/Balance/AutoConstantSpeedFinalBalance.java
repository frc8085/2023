// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Shared.Balance;

import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoConstantSpeedFinalBalance extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_drive;
  private double m_speed = AutoConstants.kFinalBalanceSpeed;
  private boolean isBalanced = false;

  private double autoEndTime = 14.8; // Lock before Auto ends
  private boolean timeIsUp = false;

  public AutoConstantSpeedFinalBalance(DriveSubsystem drive) {
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

    // Get the elapsed time since the start of Auto
    double autoElapsedTime = Robot.getElapsedTime();

    // Unsure if the clock resets at end of Auto
    // Just to be safe, don't let this turn back to false
    timeIsUp = timeIsUp || autoElapsedTime >= autoEndTime;

    double currentPitch = m_drive.getPitch().getDegrees();
    isBalanced = Math.abs(currentPitch) <= 1.5;

    if (timeIsUp || isBalanced) {
      m_drive.lock();
    } else {
      m_drive.drive(
          m_speed,
          currentPitch > 0 ? -1 : 1,
          0,
          0,
          false,
          false);
    }

    SmartDashboard.putBoolean("FINAL BALANCED", isBalanced);
  }

  // Stop driving when the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
  }

  // TODO: Need to test in Practice mode that
  // Driver regains control in Teleop
  // Because this Command never ends
  @Override
  public boolean isFinished() {
    return false;
  }
}
