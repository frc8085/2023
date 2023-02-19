// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.ElevatorConstants;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Drive the given distance straight (negative values go backwards). Uses a
 * local PID controller to run a simple PID loop that is only enabled while t
 * is command is running. The input is the averaged values of the left and right
 * encoders.
 */
public class KeepAltitude extends PIDCommand {
  private final Elevator m_elevator;

  static double kP = 0.1;
  static double kI = 0;
  static double kD = 0.001;

  /**
   * Create a new KeepAltitude command.
   *
   * @param distance The distance to move (degrees)
   */
  public KeepAltitude(DoubleSupplier desiredAltitude, Elevator elevator) {
    super(
        new PIDController(kP, kI, kD),
        elevator::getCurrentAltitudeAngle,
        desiredAltitude.getAsDouble(),
        output -> elevator.setElevator(output));

    m_elevator = elevator;
    addRequirements(m_elevator);
    getController().setTolerance(ElevatorConstants.kAltitudePositionTolerance);

    SmartDashboard.putNumber("Altitude Angle to maintain",
        desiredAltitude.getAsDouble());

  }

  @Override
  public void execute() {

    super.execute();
    getController().setSetpoint(m_elevator.getCurrentAltitudeAngle());
    SmartDashboard.putNumber("Altitude SETPOINT",
        getController().getSetpoint());

  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    // Get everything in a safe starting state.
    // m_elevator.startKeepingAltitude();
    super.initialize();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    // return getController().atSetpoint();

    // If we still need to keep altitude, then restart the PID
    boolean resetSetpoint = getController().atSetpoint() &&
        m_elevator.IsElevatorKeepingAltitude();

    if (resetSetpoint) {
      new SequentialCommandGroup(
          new WaitCommand(0.25),
          new InstantCommand(() -> getController().setSetpoint(m_elevator.getCurrentAltitudeAngle())));
      return false;
    } else {
      return getController().atSetpoint();
    }
  }
}