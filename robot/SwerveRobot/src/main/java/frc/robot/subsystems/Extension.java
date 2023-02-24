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
  private final CANSparkMax m_extensionMotor = new CANSparkMax(ExtensionConstants.kExtensionMotorPort,
      MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder m_extensionEncoder = m_extensionMotor.getEncoder();

  // Limit Switches
  private SparkMaxLimitSwitch m_extensionLimit;
  private SparkMaxLimitSwitch m_retractionLimit;

  // PID
  private SparkMaxPIDController m_extensionPIDController = m_extensionMotor.getPIDController();
  static double kPExtension = .25;
  static double kIExtension = 0;
  static double kDExtension = 0;

  public boolean ExtensionIsInTravelPosition() {
    return isRetractionLimitHit();
  }

  public Extension() {

    m_extensionMotor.setIdleMode(IdleMode.kBrake);
    m_extensionMotor.setSmartCurrentLimit((SubsystemMotorConstants.kMotorCurrentLimit));
    m_extensionMotor.setOpenLoopRampRate(ExtensionConstants.kExtensionRampRate);

    m_extensionPIDController.setFeedbackDevice(m_extensionEncoder);
    m_extensionPIDController.setP(kPExtension, 0);
    m_extensionPIDController.setI(kIExtension, 0);
    m_extensionPIDController.setD(kDExtension, 0);
    m_extensionPIDController.setOutputRange(-0.5, 0.5);

    // TODO. What should these values be?
    m_extensionPIDController.setSmartMotionMaxAccel(0.5, 0);
    m_extensionPIDController.setSmartMotionMaxVelocity(0.5, 0);

    /**
     * A SparkMaxLimitSwitch object is constructed using the getForwardLimitSwitch()
     * or
     * on which direction you would like to limit
     * 
     * Limit switches can be configured to one of two polarities:
     * com.revrobotics.SparkMaxLimitSwitch.SparkMaxLimitSwitch.Type.kNormallyOpen
     * com.revrobotics.SparkMaxLimitSwitch.SparkMaxLimitSwitch.Type.kNormallyClosed
     */
    m_extensionLimit = m_extensionMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_retractionLimit = m_extensionMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.

    m_extensionMotor.burnFlash();
  }

  /** The log method puts interesting information to the SmartDashboard. */
  public void log() {
    SmartDashboard.putNumber("Extension Raw encoder read", m_extensionEncoder.getPosition());
    SmartDashboard.putBoolean("Fully Extended", m_extensionLimit.isPressed());
    SmartDashboard.putBoolean("Fully Retracted", m_retractionLimit.isPressed());
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
    m_extensionEncoder.setPosition(0);
  }

  // Reset the Extension Encoder when the Retraction Limit is pressed
  public boolean isRetractionLimitHit() {
    return m_retractionLimit.isPressed();
  }

  public void resetExtensionEncoderAtRetractionLimit() {
    if (isRetractionLimitHit()) {
      m_extensionEncoder.setPosition(1);
    }
  }

  private boolean startingPositionRetractionTravelLimit = false;

  public void moveToStartingPosition() {
    if (!isRetractionLimitHit()) {
      retractExtension();
    } else
      stopExtension();
    startingPositionRetractionTravelLimit = true;
  }

  public boolean startingPositionRetractionTravelLimit() {
    return startingPositionRetractionTravelLimit;
  }

  /** ELEVATOR Extension **/
  // Run the elevator Extension motor forward
  public void extendExtension() {
    m_extensionMotor.set(ExtensionConstants.kExtensionSpeed);
  }

  // Run the elevator Extension motor in reverse
  public void retractExtension() {
    m_extensionMotor.set(-ExtensionConstants.kExtensionSpeed);
  }

  // Stop the elevator Extension
  public void stopExtension() {
    m_extensionMotor.set(0);
  }

  // Returns the current position of the Extension
  public double getCurrentExtensionPosition() {
    return m_extensionEncoder.getPosition();
  }

  // Set a variable speed
  public void setExtension(double speed) {
    m_extensionMotor.set(speed * ExtensionConstants.kMaxExtensionSpeedMetersPerSecond);
  }

  // Maintain Position
  public void keepPosition(double position) {
    m_extensionPIDController.setReference(position, ControlType.kPosition);
    SmartDashboard.putNumber("Desired Extension position", position);
  }
}
