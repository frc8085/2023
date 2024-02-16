package frc.robot.commands.Autos.Shared.Balance;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class AutoPitchRollBalance extends Command {
  private final DriveSubsystem m_drive;

  private static final double speedMetersPerSecond = 0.2;
  private static final double pitchToleranceDegress = 3.0;

  private double pitchDegrees;

  private double autoEndTime = 14.8; // Lock before Auto ends
  private boolean timeIsUp = false;

  public AutoPitchRollBalance(DriveSubsystem drive) {
    m_drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    pitchDegrees = Double.POSITIVE_INFINITY;
  }

  @Override
  public void execute() {
    // Get the elapsed time since the start of Auto
    double autoElapsedTime = Robot.getElapsedTime();

    // Unsure if the clock resets at end of Auto
    // Just to be safe, don't let this turn back to false
    timeIsUp = timeIsUp || autoElapsedTime >= autoEndTime;

    // Calculate charge station angle and velocity
    pitchDegrees = m_drive.getRotation().getCos() * m_drive.getPitch().getDegrees()
        + m_drive.getRotation().getSin() * m_drive.getRoll().getDegrees();

    boolean isBalanced = Math.abs(pitchDegrees) < pitchToleranceDegress;

    // Drive
    if (timeIsUp || isBalanced) {
      m_drive.lock();
    } else {
      m_drive.drive(
          speedMetersPerSecond,
          (pitchDegrees > 0 ? -1 : 1),
          0,
          0,
          false,
          false);
    }

  }

  @Override
  public void end(boolean interrupted) {
    m_drive.lock();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(pitchDegrees) < pitchToleranceDegress;
  }
}