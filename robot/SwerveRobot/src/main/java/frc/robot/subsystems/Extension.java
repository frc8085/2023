// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ExtensionConstants;
import static frc.robot.Constants.SubsystemMotorConstants;

public class Extension extends SubsystemBase {
  /** Creates a new Extension. */

  // Extension motors
  private final CANSparkMax m_ExtensionMotor = new CANSparkMax(ExtensionConstants.kExtensionMotorPort,
      MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder m_ExtensionEncoder = m_ExtensionMotor.getEncoder();

  // Limit Switches
  private SparkMaxLimitSwitch m_ExtensionLimit;
  private SparkMaxLimitSwitch m_RetractionLimit;

  // PID
  private SparkMaxPIDController m_ExtensionPIDController = m_ExtensionMotor.getPIDController();
  static double kPExtension = .25;
  static double kIExtension = 0;
  static double kDExtension = 0;

  public boolean ExtensionIsInTravelPosition() {
    return isRetractionLimitHit();
  }

  public Extension() {

    m_ExtensionMotor.setIdleMode(IdleMode.kBrake);
    m_ExtensionMotor.setSmartCurrentLimit((SubsystemMotorConstants.kMotorCurrentLimit));
    m_ExtensionMotor.setOpenLoopRampRate(ExtensionConstants.kExtensionRampRate);

    m_ExtensionPIDController.setFeedbackDevice(m_ExtensionEncoder);
    m_ExtensionPIDController.setP(kPExtension, 0);
    m_ExtensionPIDController.setI(kIExtension, 0);
    m_ExtensionPIDController.setD(kDExtension, 0);
    m_ExtensionPIDController.setOutputRange(-0.5, 0.5);

    // TODO. What should these values be?
    m_ExtensionPIDController.setSmartMotionMaxAccel(0.5, 0);
    m_ExtensionPIDController.setSmartMotionMaxVelocity(0.5, 0);

    /**
     * A SparkMaxLimitSwitch object is constructed using the getForwardLimitSwitch()
     * or
     * on which direction you would like to limit
     * 
     * Limit switches can be configured to one of two polarities:
     * com.revrobotics.SparkMaxLimitSwitch.SparkMaxLimitSwitch.Type.kNormallyOpen
     * com.revrobotics.SparkMaxLimitSwitch.SparkMaxLimitSwitch.Type.kNormallyClosed
     */
    m_ExtensionLimit = m_ExtensionMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_RetractionLimit = m_ExtensionMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.

    m_ExtensionMotor.burnFlash();
  }

  /** The log method puts interesting information to the SmartDashboard. */
  public void log() {
    SmartDashboard.putNumber("Extension Raw encoder read", m_ExtensionEncoder.getPosition());
    SmartDashboard.putBoolean("Fully Extended", m_ExtensionLimit.isPressed());
    SmartDashboard.putBoolean("Fully Retracted", m_RetractionLimit.isPressed());
    SmartDashboard.putBoolean("Extension Travel Position", ExtensionIsInTravelPosition());

    SmartDashboard.putNumber("Current position", getCurrentExtensionPosition());
  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();
    resetExtensionEncoderAtRetractionLimit();
    ExtensionIsInTravelPosition();
  }

  /** Resets the Extension encoder to currently read a position of 0. */
  public void reset() {
    m_ExtensionEncoder.setPosition(0);
  }

  // Reset the Extension Encoder when the Retraction Limit is pressed
  public boolean isRetractionLimitHit() {
    return m_RetractionLimit.isPressed();
  }

  public void resetExtensionEncoderAtRetractionLimit() {
    if (isRetractionLimitHit()) {
      m_ExtensionEncoder.setPosition(1);
    }
  }

  /** ELEVATOR Extension **/
  // Run the elevator Extension motor forward
  public void extendExtension() {
    m_ExtensionMotor.set(ExtensionConstants.kExtensionSpeed);
  }

  // Run the elevator Extension motor in reverse
  public void retractExtension() {
    m_ExtensionMotor.set(-ExtensionConstants.kExtensionSpeed);
  }

  // Stop the elevator Extension
  public void stopExtension() {
    m_ExtensionMotor.set(0);
  }

  // Returns the current position of the Extension
  public double getCurrentExtensionPosition() {
    return m_ExtensionEncoder.getPosition();
  }

  // Set a variable speed
  public void setExtension(double speed) {
    m_ExtensionMotor.set(speed * ExtensionConstants.kMaxExtensionSpeedMetersPerSecond);
  }

  // Maintain Position
  public void keepPosition(double position) {
    m_ExtensionPIDController.setReference(position, ControlType.kPosition);
    SmartDashboard.putNumber("Desired Extension position", position);
  }
}
