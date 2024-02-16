package frc.robot.commands.Autos.Shared.Balance;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Final balance for main character auto
 */
public class AutoLock extends Command {
  private final DriveSubsystem m_drive;

  private double maxSpeed = 0.11;
  private double setpoint = 0; // Balance = 0 degrees
  private double tolerance = 6; // tolerance

  private double lockDuration;
  private double lockNormalDuration = 0.5;
  private double firstLockDuration = 0.5;

  private double decreasingSpeed = maxSpeed;

  private Timer lockTimer = new Timer();

  private double autoEndTime = 14.8; // Lock before Auto ends
  private boolean timeIsUp = false;

  // Creates a new flat moving average filter
  // Average will be taken over the last 10 samples
  LinearFilter pitchFilter = LinearFilter.movingAverage(10);
  LinearFilter absPitchFilter = LinearFilter.movingAverage(10);

  public AutoLock(DriveSubsystem drive) {
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    super.initialize();
    lockTimer.reset();
    lockTimer.start();
    lockDuration = firstLockDuration;
    maxSpeed = 0.11;
  }

  @Override
  public void execute() {
    super.execute();

    // Get the elapsed time since the start of Auto
    double autoElapsedTime = Robot.getElapsedTime();

    // Unsure if the clock resets at end of Auto
    // Just to be safe, don't let this turn back to false
    timeIsUp = timeIsUp || autoElapsedTime >= autoEndTime;

    // Get the pitch degrees averaged over 10 samples
    double averagePitch = absPitchFilter.calculate(Math.abs(m_drive.getPitch().getDegrees()));
    double instantaneousPitch = m_drive.getPitch().getDegrees();

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

    System.out.println(Robot.getElapsedTime() + " Average ABS Pitch: " + averagePitch);
    System.out.println(Robot.getElapsedTime() + " isBalanced =" + isBalanced);

    double timeSinceLock = lockTimer.get();

    // If we just locked the wheels
    // Wait for the lock duration before deciding whether to unlock and move
    if ((timeSinceLock > 0) && (timeSinceLock <= lockDuration)) {
      m_drive.lock();
    }
    // If the time lock runs out, check if it's time to lock
    // Which means we are either balanced or time's running out
    else if (timeToLock) {
      System.out.println(Robot.getElapsedTime() + " timeToLock TRUE");

      m_drive.lock();
      lockTimer.restart();
      lockDuration = lockNormalDuration;
      maxSpeed = 0.11;
    } else {
      // Otherwise, drive up or down depending on the current pitch reading.
      // Drive at a decreasing speed over time
      System.out.println(Robot.getElapsedTime() + " Driving -  instantaneousPitch: " + instantaneousPitch);

      m_drive.drive(
          // decreasingSpeed,
          maxSpeed,
          instantaneousPitch > 0 ? -1 : 1,
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
