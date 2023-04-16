package frc.robot.commands.Autos.Shared.Balance;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Final balance for main character auto
 */
public class AutoDecreasingSpeedBalance extends CommandBase {
  private final DriveSubsystem m_drive;

  private double maxSpeed = 0.10;
  private double setpoint = 0; // Balance = 0 degrees
  private double tolerance = 3; // 3 Degree tolerance

  private double decreasingSpeed = maxSpeed;

  private Timer localTimer = new Timer();

  private double autoEndTime = 14.8; // Lock before Auto ends
  boolean timeIsUp = false;

  // Creates a new flat moving average filter
  // Average will be taken over the last 10 samples
  LinearFilter pitchFilter = LinearFilter.movingAverage(10);
  LinearFilter absPitchFilter = LinearFilter.movingAverage(10);

  public AutoDecreasingSpeedBalance(DriveSubsystem drive) {
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    super.initialize();
    localTimer.reset();
    localTimer.start();
  }

  @Override
  public void execute() {
    super.execute();

    // Get the elapsed time since the start of Auto
    double autoElapsedTime = Robot.getElapsedTime();

    // Get the elapsed time since our last local timer check
    double elapsedLocalTime = localTimer.get();

    // Unsure if the clock resets at end of Auto
    // Just to be safe, don't let this turn back to false
    timeIsUp = timeIsUp || autoElapsedTime >= autoEndTime;

    // Get the pitch degrees averaged over 10 samples
    double averagePitch = pitchFilter.calculate(m_drive.getPitch().getDegrees());
    double momentPitch = m_drive.getPitch().getDegrees();

    // Determine if we are balanced
    boolean isBalanced = averagePitch <= (setpoint + tolerance);

    // Lock wheels if we're balanced or time is up
    boolean timeToLock = timeIsUp || isBalanced;

    /*
     * Slow down from max speed as time elapses
     * 
     * For reference, our original "FinalBalance" used
     * AutoConstants.kFinalBalanceSpeed = 0.0525
     * 
     * T <= 7s | S = 0.10 <-- Dont' go above max speed
     * ...
     * T = 10s | S = 0.10 / ( 10 / 7 ) = 0.070
     * T = 11s | S = 0.10 / ( 11 / 7 ) = 0.064
     * T = 12s | S = 0.10 / ( 12 / 7 ) = 0.058
     * T = 13s | S = 0.10 / ( 13 / 7 ) = 0.054
     * T = 14s | S = 0.10 / ( 14 / 7 ) = 0.050
     * T = 14.8s | S = 0.10 / ( 14.8 / 7 ) = 0.047 <-- Lock time
     * 
     */
    if (elapsedLocalTime > 0.5) {
      decreasingSpeed = Math.min(maxSpeed, maxSpeed / (autoElapsedTime / 7));
      localTimer.reset();
    }

    if (timeToLock) {
      m_drive.lock();
    } else {
      // Otherwise, drive up or down depending on the current pitch reading.
      // Drive at a decreasing speed over time
      m_drive.drive(
          decreasingSpeed,
          momentPitch > 0 ? -1 : 1,
          0,
          0,
          false,
          false);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }
}
