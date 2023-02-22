// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AltitudeConstants;

import static frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private Altitude m_altitude;

  /** Creates a new Intake. */

  // Intake motor
  private final CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakePort, MotorType.kBrushless);

  // Intake Encoder
  private final RelativeEncoder m_intakeEncoder = m_intakeMotor.getEncoder();

  // Add velocity PID for Intake Motor
  private SparkMaxPIDController m_intakePIDController = m_intakeMotor.getPIDController();
  static double kPIntake = 0;
  static double kIIntake = 0;
  static double kDIntake = 0;
  static double kMaxIntakeSpeed = 1;
  static double kMinIntakeSpeed = -1;

  // Attempt to write position encoder stall detection
  static double intakeEncoder = 0;
  static double kIntakeEncoderTolerance = 1;

  public double CurrentIntakeEncoderPosition() {
    return m_intakeEncoder.getPosition();
  }

  public Intake(Altitude Altitude) {
    m_altitude = Altitude;

    public boolean altitudeIntakeRunPosition() {
      return m_altitude.getCurrentAltitude() < AltitudeConstants.kAltitudeIntakePosition
          + AltitudeConstants.kAltitudePositionTolerance;
    }
  
    m_intakeMotor.setOpenLoopRampRate(IntakeConstants.kRampRate);

    m_intakePIDController.setFeedbackDevice(m_intakeEncoder);
    m_intakePIDController.setP(kPIntake, 0);
    m_intakePIDController.setI(kIIntake, 0);
    m_intakePIDController.setD(kDIntake, 0);
    m_intakePIDController.setOutputRange(kMinIntakeSpeed, kMaxIntakeSpeed);

    m_intakeMotor.burnFlash();
  }

  // Intake PID functions
  public void pickupObject() {
    m_intakePIDController.setReference(IntakeConstants.kIntakeConeSpeed, ControlType.kVelocity);
  }

  public void holdObject() {
    m_intakePIDController.setReference(0, ControlType.kVelocity);
  }

  public void ejectObject() {
    m_intakePIDController.setReference(IntakeConstants.kEjectSpeed, ControlType.kVelocity);
  }

  private boolean bumpObject = false;

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

  // Log info for smart dashboard
  public void log() {
    SmartDashboard.putNumber("Intake P Gain", kPIntake);
    SmartDashboard.putNumber("Intake I Gain", kIIntake);
    SmartDashboard.putNumber("Intake D Gain", kDIntake);
    SmartDashboard.putNumber("Intake Max Speed", kMaxIntakeSpeed);
    SmartDashboard.putNumber("Intake Min Speed", kMinIntakeSpeed);

  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();
    // Read PID Coefficients from SmartDashboard
    double pIntake = SmartDashboard.getNumber("Intake P Gain", 0);
    double iIntake = SmartDashboard.getNumber("Intake I Gain", 0);
    double dIntake = SmartDashboard.getNumber("Intake D Gain", 0);
    double maxIntake = SmartDashboard.getNumber("Intake Max Speed", 0);
    double minIntake = SmartDashboard.getNumber("Intake Min Speed ", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((pIntake != kPIntake)) {
      m_intakePIDController.setP(pIntake);
      kPIntake = pIntake;
    }
    if ((iIntake != kIIntake)) {
      m_intakePIDController.setI(iIntake);
      kIIntake = iIntake;
    }
    if ((dIntake != kDIntake)) {
      m_intakePIDController.setD(dIntake);
      kDIntake = dIntake;
    }
    if ((maxIntake != kMaxIntakeSpeed) || (minIntake != kMinIntakeSpeed)) {
      m_intakePIDController.setOutputRange(kMinIntakeSpeed, kMaxIntakeSpeed);
      kMinIntakeSpeed = minIntake;
      kMaxIntakeSpeed = maxIntake;
    }

  }

  // check if Intake Motor has picked up object
  public void runIntakeUntilObjectPickedUp(double intakeEncoder) {
    if ((CurrentIntakeEncoderPosition() > intakeEncoder + kIntakeEncoderTolerance)) {
      pickupObject();
      intakeEncoder = m_intakeEncoder.getPosition();
    } else {
      holdObject();
    }

  }
}
