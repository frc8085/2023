// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Drive until we balance. Uses a local PID controller to
 * run a simple PID loop that is only enabled while this command is running.
 * The input is the current pitch reading from our Gyro
 */
public class AutoEngage extends PIDCommand {
  private boolean TUNING_MODE = true;
  private final DriveSubsystem m_drive;
  private static double desiredPitchDegree = 0;
  private static double pitchDegreesTolerance = 1;

  static double kP = 0.01;
  static double kI = 0;
  static double kD = 0.001;

  /**
   * Create a new AutoEngage command.
   */
  public AutoEngage(DriveSubsystem drive) {
    super(new PIDController(kP, kI, kD),
        // Close loop on pitch degrees
        drive::getPitch,
        // Set reference to target
        desiredPitchDegree,
        // Pipe output to drive robot
        output -> drive.drive(
            false,
            // The PID output to control our speed
            output,
            // If reading positive pitch, drive backwards.
            // If reading negative pitch, drive forwards
            Math.signum(-drive.getPitch()),
            0,
            0,
            true,
            false));

    // Require the drive
    m_drive = drive;
    addRequirements(m_drive);

    // Set desired pitch tolerance in degrees
    getController().setTolerance(pitchDegreesTolerance);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    super.initialize();
    // Get everything in a safe starting state.
    m_drive.resetOdometry(new Pose2d());
    m_drive.zeroHeading();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    // TODO: Test. If we fail to balance, make sure we still lock our wheels in Auto
    boolean timeToLock = Timer.getMatchTime() < 1.5;
    return timeToLock || getController().atSetpoint();
  }

  // Allows us to fine tune PID from the dashboard
  private void readPIDTuningFromDashboard() {
    // Read PID Coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("Balance P Gain", kP);
    double i = SmartDashboard.getNumber("Balance I Gain", kI);
    double d = SmartDashboard.getNumber("Balance D Gain", kD);

    // If PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != kP)) {
      getController().setP(p);
      kP = p;
    }
    if ((i != kI)) {
      getController().setI(i);
      kI = i;
    }
    if ((d != kD)) {
      getController().setD(d);
      kD = d;
    }
  }

}
