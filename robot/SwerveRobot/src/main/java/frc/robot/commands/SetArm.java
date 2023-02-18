// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.ArmConstants;

import java.util.function.DoubleSupplier;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 * Drive the given distance straight (negative values go backwards). Uses a
 * local PID controller to run a simple PID loop that is only enabled while t
 * is command is running. The input is the averaged values of the left and right
 * encoders.
 */
public class SetArm extends PIDCommand {
  private final Arm m_arm;

  static double kP = 0.1;
  static double kI = 0;
  static double kD = 0.001;

  /**
   * Create a new SetArm command.
   *
   * @param distance The distance to move (degrees)
   */
  public SetArm(double armValue, Arm arm) {
    super(
        new PIDController(kP, kI, kD),
        arm::getCurrentArmPosition,
        armValue,
        output -> arm.setArm(output));

    m_arm = arm;
    addRequirements(m_arm);
    getController().setTolerance(ArmConstants.kArmPositionTolerance);

    // SmartDashboard.putNumber("Altitude to maintain",
    // armValue.getAsDouble());

  }

  @Override
  public void execute() {
    super.execute();
    // SmartDashboard.putNumber("Desired Position", m_Arm.)
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    // Get everything in a safe starting state.
    m_arm.reset();
    super.initialize();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
