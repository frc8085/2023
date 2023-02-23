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
import frc.robot.Constants.ExtensionConstants;

import static frc.robot.Constants.AltitudeConstants;
import static frc.robot.Constants.SubsystemMotorConstants;

public class Altitude extends SubsystemBase {
  private Extension m_extension;
  /** Creates a new Altitude. */

  // Altitude motors
  private final CANSparkMax m_altitudeMotor = new CANSparkMax(AltitudeConstants.kAltitudeMotorPort,
      MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder m_altitudeEncoder = m_altitudeMotor.getEncoder();

  // Limit Switches
  private SparkMaxLimitSwitch m_altitudeTopLimit;
  private SparkMaxLimitSwitch m_altitudeBottomLimit;

  // PID
  private SparkMaxPIDController m_altitudePIDController = m_altitudeMotor.getPIDController();
  static double kPAltitude = 8;
  static double kIAltitude = 0;
  static double kDAltitude = 0;
  static double kIzAltitude = 0;
  static double kFFAltitude = 0;
  static double kMaxOutputAltitude = .6;
  static double kMinOutputAltitude = -.6;

  public boolean AltitudeIsInTravelPosition() {
    return isAltitudeTopLimitHit();
  }

  public boolean isWithinSafeExtensionLimit() {
    return m_extension.getCurrentExtensionPosition() < ExtensionConstants.kExtensionPositionIntakeOut;
  }

  public Altitude(Extension Extension) {
    m_extension = Extension;
    m_altitudeMotor.setIdleMode(IdleMode.kBrake);
    m_altitudeMotor.setSmartCurrentLimit(SubsystemMotorConstants.kMotorCurrentLimit);
    m_altitudeMotor.setOpenLoopRampRate(AltitudeConstants.kAltitudeRampRate);

    m_altitudePIDController.setFeedbackDevice(m_altitudeEncoder);
    m_altitudePIDController.setP(kPAltitude, 0);
    m_altitudePIDController.setI(kIAltitude, 0);
    m_altitudePIDController.setD(kDAltitude, 0);
    m_altitudePIDController.setOutputRange(kMinOutputAltitude, kMaxOutputAltitude);

    // TODO. What should these values be?
    m_altitudePIDController.setSmartMotionMaxAccel(0.5, 0);
    m_altitudePIDController.setSmartMotionMaxVelocity(0.5, 0);

    /**
     * A SparkMaxLimitSwitch object is constructed using the getForwardLimitSwitch()
     * or
     * getReverseLimitSwitch() method on an existing CANSparkMax object, depending
     * on which direction you would like to limit
     * 
     * Limit switches can be configured to one of two polarities:
     * com.revrobotics.SparkMaxLimitSwitch.SparkMaxLimitSwitch.Type.kNormallyOpen
     * com.revrobotics.SparkMaxLimitSwitch.SparkMaxLimitSwitch.Type.kNormallyClosed
     */
    m_altitudeTopLimit = m_altitudeMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_altitudeBottomLimit = m_altitudeMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_altitudeMotor.burnFlash();

  }

  /** The log method puts interesting information to the SmartDashboard. */
  public void log() {
    SmartDashboard.putNumber("Altitude Raw encoder read", m_altitudeEncoder.getPosition());
    SmartDashboard.putBoolean("Altitude at Top Position", m_altitudeTopLimit.isPressed());
    SmartDashboard.putBoolean("Altitude at Bottom Position", m_altitudeBottomLimit.isPressed());
    SmartDashboard.putBoolean("Altitude is in Travel Position", AltitudeIsInTravelPosition());

    SmartDashboard.putNumber("Current altitude", getCurrentAltitude());

    SmartDashboard.putNumber("Altitude P Gain", kPAltitude);
    SmartDashboard.putNumber("Altitude I Gain", kIAltitude);
    SmartDashboard.putNumber("Altitude D Gain", kDAltitude);
    SmartDashboard.putNumber("Altitude I Zone", kIzAltitude);
    SmartDashboard.putNumber("Altitude Feed Forward", kFFAltitude);
    SmartDashboard.putNumber("Altitude Max Output", kMaxOutputAltitude);
    SmartDashboard.putNumber("Altitude Min Output", kMinOutputAltitude);
    SmartDashboard.putNumber("Altitude Set Rotations", 0);

  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();

    resetAltitudeEncoderAtTopLimit();
    AltitudeIsInTravelPosition();

    // Read PID Coefficients from SmartDashboard
    double pAltitude = SmartDashboard.getNumber("Altitude P Gain", 0);
    double iAltitude = SmartDashboard.getNumber("Altitude I Gain", 0);
    double dAltitude = SmartDashboard.getNumber("Altitude D Gain", 0);
    double izAltitude = SmartDashboard.getNumber("Altitude I Zone", 0);
    double ffAltitude = SmartDashboard.getNumber("Altitude Feed Forward", 0);
    double maxAltitude = SmartDashboard.getNumber("Altitude Max Output", 0);
    double minAltitude = SmartDashboard.getNumber("Altitude Min Output", 0);
    double positionAltitude = SmartDashboard.getNumber("Set Position", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((pAltitude != kPAltitude)) {
      m_altitudePIDController.setP(pAltitude);
      kPAltitude = pAltitude;
    }
    if ((iAltitude != kIAltitude)) {
      m_altitudePIDController.setI(iAltitude);
      kIAltitude = iAltitude;
    }
    if ((dAltitude != kDAltitude)) {
      m_altitudePIDController.setD(dAltitude);
      kDAltitude = dAltitude;
    }
    if ((izAltitude != kIzAltitude)) {
      m_altitudePIDController.setIZone(izAltitude);
      kIzAltitude = izAltitude;
    }
    if ((ffAltitude != kFFAltitude)) {
      m_altitudePIDController.setFF(ffAltitude);
      kFFAltitude = ffAltitude;
    }
    if ((maxAltitude != kMaxOutputAltitude) || (minAltitude != kMinOutputAltitude)) {
      m_altitudePIDController.setOutputRange(minAltitude, maxAltitude);
      kMinOutputAltitude = minAltitude;
      kMaxOutputAltitude = maxAltitude;
    }

  }

  /** Resets the Altitude encoder to currently read a position of 0. */
  public void reset() {
    m_altitudeEncoder.setPosition(0);
  }

  // Reset the altitude Encoder when the top limit is pressed
  public boolean isAltitudeTopLimitHit() {
    return m_altitudeTopLimit.isPressed() == true;
  }

  public void resetAltitudeEncoderAtTopLimit() {
    if (isAltitudeTopLimitHit()) {
      m_altitudeEncoder.setPosition(-0.1);
    }
  };

  /** ALTITUDE **/
  // Run the Altitude motor forward
  public void raiseAltitude() {
    m_altitudeMotor.set(AltitudeConstants.kAltitudeSpeed);
  }

  // Run the Altitude motor in reverse
  public void lowerAltitude() {
    m_altitudeMotor.set(-AltitudeConstants.kAltitudeSpeed);
  }

  // Stop the Altitude
  public void stopAltitude() {
    m_altitudeMotor.set(0);
  }

  // Returns the current altitude
  public double getCurrentAltitude() {
    return m_altitudeEncoder.getPosition();
  }

  // Returns the current altitude in degrees
  // change altitude encoder readings to degrees angle*4.75/80
  // making assumption that -4.75 is -80 degrees
  public double getCurrentAltitudeAngle() {
    return (m_altitudeEncoder.getPosition() / 4.75 * 80);
  }

  // Set a variable speed to move to a position
  public void setAltitude(double speed) {
    if (!isWithinSafeExtensionLimit()) {
      m_altitudeMotor.set(speed * AltitudeConstants.kMaxLimitedAltitudeSpeedMetersPerSecond);
    } else {
      m_altitudeMotor.set(speed * AltitudeConstants.kMaxAltitudeSpeedMetersPerSecond);
    }
  }

  // Maintain position
  public void keepPosition(double positionAltitude) {
    m_altitudePIDController.setReference(positionAltitude, ControlType.kPosition);
    SmartDashboard.putNumber("Altitude Desired position", positionAltitude);
  }
}
