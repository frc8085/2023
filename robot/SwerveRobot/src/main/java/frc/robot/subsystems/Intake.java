// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  // Intake motor
  private final CANSparkMax m_IntakeMotor = new CANSparkMax(IntakeConstants.kIntakePort, MotorType.kBrushless);

  public Intake() {
    m_IntakeMotor.setOpenLoopRampRate(IntakeConstants.kRampRate);
  }

  // Run the intake forward at the CONE speed
  public void intakeCone() {
    m_IntakeMotor.set(IntakeConstants.kIntakeConeSpeed);
  }

  // Run the intake forward at the CUBE speed
  public void intakeCube() {
    m_IntakeMotor.set(IntakeConstants.kIntakeCubeSpeed);
  }

  // Run the intake reverse to eject cargo
  public void eject() {
    m_IntakeMotor.set(-IntakeConstants.kEjectSpeed);
  }

  // Stop the intake
  public void stopIntake() {
    m_IntakeMotor.set(0);
  }

}
