// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Altitude;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 * Drive the given distance straight (negative values go backwards). Uses a
 * local PID controller to
 * run a simple PID loop that is only enabled while this command is running. The
 * input is the
 * averaged values of the left and right encoders.
 */
public class RaiseLower extends PIDCommand {
  private final Altitude m_altitude;

  static double kP = .5;
  static double kI = .0001;
  static double kD = .1;

  /**
   * Create a new TurnToDegreeGyro command.
   *
   * @param distance The distance to drive (inches)
   */
  public RaiseLower(Altitude altitude, double position) {
    super(new PIDController(kP, kI, kD),
        // Close loop on heading
        altitude::getCurrentAltitude,
        // Set reference to target
        position,
        // Pipe output to turn robot
        output -> altitude.moveAltitude(output));

    // Require the drive
    m_altitude = altitude;
    addRequirements(m_altitude);

    getController().setTolerance(.5);
  }

  @Override
  public void execute() {
    super.execute();
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    super.initialize();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
