package frc.robot.commands.Autos.Shared.Balance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoPitchVelocityBalance extends CommandBase {
  private final DriveSubsystem m_drive;

  private static final double speedMetersPerSecond = 0.2;
  private static final double pitchToleranceDegress = 3.0;
  private static final double velocityToleranceDegreesPerSecond = 8.0;

  private double pitchDegrees;

  public AutoPitchVelocityBalance(DriveSubsystem drive) {
    m_drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    pitchDegrees = Double.POSITIVE_INFINITY;
  }

  @Override
  public void execute() {
    // Calculate charge station angle and velocity
    pitchDegrees = m_drive.getRotation().getCos() * m_drive.getPitch().getDegrees()
        + m_drive.getRotation().getSin() * m_drive.getRoll().getDegrees();
    double angleVelocityDegreesPerSec = m_drive.getRotation().getCos()
        * m_drive.getPitchVelocity()
        + m_drive.getRotation().getSin() * m_drive.getRollVelocity();
    boolean shouldStop = (pitchDegrees < 0.0 && angleVelocityDegreesPerSec > velocityToleranceDegreesPerSecond)
        || (pitchDegrees > 0.0
            && angleVelocityDegreesPerSec < -velocityToleranceDegreesPerSecond);

    // Drive
    if (shouldStop) {
      m_drive.stop();
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