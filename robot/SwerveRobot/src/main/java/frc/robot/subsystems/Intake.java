// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemMotorConstants;

import static frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  // Intake motor
  private final CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakePort, MotorType.kBrushless);

  public Intake() {
    m_intakeMotor.setOpenLoopRampRate(IntakeConstants.kRampRate);
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
    m_intakeMotor.setSmartCurrentLimit(SubsystemMotorConstants.kMotorCurrentLimit550);

    m_intakeMotor.burnFlash();

  }

  // Run the intake forward at the CONE speed
  public void intakeCone() {
    m_intakeMotor.set(IntakeConstants.kIntakeConeSpeed);
  }

  // Run the intake forward at the CUBE speed
  public void intakeCube() {
    m_intakeMotor.set(IntakeConstants.kIntakeCubeSpeed);
  }

  // Run the intake reverse to eject cargo
  public void eject() {
    m_intakeMotor.set(-IntakeConstants.kEjectSpeed);
  }

  // Stop the intake
  public void stopIntake() {
    m_intakeMotor.set(0);
  }

}
