// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemMotorConstants;

import static frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private boolean TUNING_MODE = true;
  private final CANSparkMax m_intakeMotor;
  private SparkMaxPIDController m_intakePIDController;
  private RelativeEncoder m_intakeEncoder;
  private double kIntakeSetPoint, kPIntake, kIIntake, kDIntake, kFFIntake, kIntakeMaxOutput, kIntakeMinOutput;

  // Determine current intake encoder position
  public double CurrentIntakeEncoderPosition() {
    return m_intakeEncoder.getPosition();
  }

  // Determine current intake encoder velocity
  public double CurrentIntakeEncoderVelocity() {
    return m_intakeEncoder.getVelocity();
  }

  /** The intake subsystem for the robot. */
  public Intake() {
    m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakePort, MotorType.kBrushless);
    m_intakeMotor.setOpenLoopRampRate(IntakeConstants.kRampRate);
    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setIdleMode(IdleMode.kCoast);
    m_intakeMotor.setSmartCurrentLimit(SubsystemMotorConstants.kMotorCurrentLimit550);
    m_intakeEncoder = m_intakeMotor.getEncoder();
    m_intakePIDController = m_intakeMotor.getPIDController();
    m_intakeMotor.burnFlash();

    // PID coefficients
    kPIntake = .5;
    kIIntake = 0;
    kDIntake = 0;
    kFFIntake = 0;
    kIntakeMaxOutput = .25;
    kIntakeMinOutput = -.25;

    // Set PID coefficients
    m_intakePIDController.setP(kPIntake);
    m_intakePIDController.setI(kIIntake);
    m_intakePIDController.setD(kDIntake);
    m_intakePIDController.setFF(kFFIntake);
    m_intakePIDController.setOutputRange(kIntakeMinOutput, kIntakeMaxOutput);

    // Add relevant displays to the Operator dashboard
    configureOperatorDashboard();

    // If we're fine-tuning PID Constants, the display them on the dashboard
    if (TUNING_MODE) {
      addPIDToDashboard();
    }

  }

  private void configureOperatorDashboard() {
    // Add the selected shooting mode to the Operator dashboard
    // SmartDashboard.putString("Cargo Mode", CargoModes.get(currentCargoMode));

    // Add the setpoint but only if in TUNINGMODE
    if (TUNING_MODE) {
      SmartDashboard.putNumber("Intake Setpoint", kIntakeSetPoint);
    }

  }

  private void addPIDToDashboard() {
    // Display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Intake P Gain", kPIntake);
    SmartDashboard.putNumber("Intake I Gain", kIIntake);
    SmartDashboard.putNumber("Intake D Gain", kDIntake);
    SmartDashboard.putNumber("Intake Max Output", kIntakeMaxOutput);
    SmartDashboard.putNumber("Intake Min Output", kIntakeMinOutput);

  }

  private void readPIDTuningFromDashboard() {

    // Read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("Intake P Gain", 0);
    double i = SmartDashboard.getNumber("Intake I Gain", 0);
    double d = SmartDashboard.getNumber("Intake D Gain", 0);
    double max = SmartDashboard.getNumber("Intake Max Output", 0);
    double min = SmartDashboard.getNumber("Intake Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != kPIntake)) {
      m_intakePIDController.setP(p);
      kPIntake = p;
    }
    if ((i != kIIntake)) {
      m_intakePIDController.setI(i);
      kIIntake = i;
    }
    if ((d != kDIntake)) {
      m_intakePIDController.setD(d);
      kDIntake = d;
    }
    if ((max != kIntakeMaxOutput) || (min != kIntakeMinOutput)) {
      m_intakePIDController.setOutputRange(min, max);
      kIntakeMinOutput = min;
      kIntakeMaxOutput = max;
    }
  }

  @Override
  public void periodic() {
    // If we're fine-tuning PID Constants, read and apply updates from the dashboard
    if (TUNING_MODE) {
      readPIDTuningFromDashboard();
      SmartDashboard.putNumber("Intake Encoder position", m_intakeEncoder.getPosition());

    }

    // add intake encoder position into log
    // logIntakeData();

  }

  public void keepVelocity(double velocity) {
    m_intakePIDController.setReference(velocity, ControlType.kVelocity);
  }

  public void holdCargo() {
    m_intakePIDController.setReference(CurrentIntakeEncoderPosition(), CANSparkMax.ControlType.kPosition);
  }

  // Open loop stuff
  // Run the intake forward at the CONE speed
  public void intakeCone() {
    m_intakeMotor.set(IntakeConstants.kIntakeConePower);
  }

  // Run the intake forward at the CUBE speed
  public void intakeCube() {
    m_intakeMotor.set(IntakeConstants.kIntakeCubePower);
  }

  // Run the intake reverse to eject CONE
  public void ejectCone() {
    m_intakeMotor.set(-IntakeConstants.kEjectConePower);
  }

  // Run the intake reverse to eject CUBE
  public void ejectCube() {
    m_intakeMotor.set(-IntakeConstants.kEjectCubePower);
  }

  // Run the intake forward to hold the CUBE speed
  public void holdCube() {
    m_intakeMotor.set(IntakeConstants.kIntakeHoldCubePower);
  }

  // Stop the intake
  public void stopIntake() {
    m_intakeMotor.set(0);
  }
}
