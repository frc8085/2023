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
import static frc.robot.Constants.AltitudeConstants;
import static frc.robot.Constants.SubsystemMotorConstants;

public class Altitude extends SubsystemBase {
  /** Creates a new Altitude. */

  // Altitude motors
  private final CANSparkMax m_AltitudeMotor = new CANSparkMax(AltitudeConstants.kAltitudeMotorPort,
      MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder m_AltitudeEncoder = m_AltitudeMotor.getEncoder();

  // Limit Switches
  private SparkMaxLimitSwitch m_AltitudeTopLimit;
  private SparkMaxLimitSwitch m_AltitudeBottomLimit;

  // PID
  private SparkMaxPIDController m_AltitudePIDController = m_AltitudeMotor.getPIDController();
  static double kPAltitude = 1;
  static double kIAltitude = 0;
  static double kDAltitude = 0;

  public boolean AltitudeIsInTravelPosition() {
    return isAltitudeTopLimitHit();
  }

  public Altitude() {
    m_AltitudeMotor.setIdleMode(IdleMode.kBrake);
    m_AltitudeMotor.setSmartCurrentLimit(SubsystemMotorConstants.kMotorCurrentLimit);
    m_AltitudeMotor.setOpenLoopRampRate(AltitudeConstants.kAltitudeRampRate);

    m_AltitudePIDController.setFeedbackDevice(m_AltitudeEncoder);
    m_AltitudePIDController.setP(kPAltitude, 0);
    m_AltitudePIDController.setI(kIAltitude, 0);
    m_AltitudePIDController.setD(kDAltitude, 0);
    m_AltitudePIDController.setOutputRange(-0.5, 0.5);

    // TODO. What should these values be?
    m_AltitudePIDController.setSmartMotionMaxAccel(0.5, 0);
    m_AltitudePIDController.setSmartMotionMaxVelocity(0.5, 0);

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
    m_AltitudeTopLimit = m_AltitudeMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_AltitudeBottomLimit = m_AltitudeMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_AltitudeMotor.burnFlash();

  }

  /** The log method puts interesting information to the SmartDashboard. */
  public void log() {
    SmartDashboard.putNumber("Altitude Raw encoder read", m_AltitudeEncoder.getPosition());
    SmartDashboard.putBoolean("Altitude at Top Position", m_AltitudeTopLimit.isPressed());
    SmartDashboard.putBoolean("Altitude at Bottom Position", m_AltitudeBottomLimit.isPressed());
    SmartDashboard.putBoolean("Altitude is in Travel Position", AltitudeIsInTravelPosition());

    SmartDashboard.putNumber("Current altitude", getCurrentAltitude());

  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();

    resetAltitudeEncoderAtTopLimit();
    AltitudeIsInTravelPosition();

  }

  /** Resets the Altitude encoder to currently read a position of 0. */
  public void reset() {
    m_AltitudeEncoder.setPosition(0);
  }

  // Reset the Arm Encoder when the Retraction Limit is pressed
  public boolean isAltitudeTopLimitHit() {
    return m_AltitudeTopLimit.isPressed() == true;
  }

  public void resetAltitudeEncoderAtTopLimit() {
    if (isAltitudeTopLimitHit()) {
      m_AltitudeEncoder.setPosition(-0.1);
    }
  };

  /** ALTITUDE **/
  // Run the Altitude motor forward
  public void raiseAltitude() {
    m_AltitudeMotor.set(AltitudeConstants.kAltitudeSpeed);
  }

  // Run the Altitude motor in reverse
  public void lowerAltitude() {
    m_AltitudeMotor.set(-AltitudeConstants.kAltitudeSpeed);
  }

  // Stop the Altitude
  public void stopAltitude() {
    m_AltitudeMotor.set(0);
  }

  // Returns the current altitude
  public double getCurrentAltitude() {
    return m_AltitudeEncoder.getPosition();
  }

  // Returns the current altitude in degrees
  // change altitude encoder readings to degrees angle*4.75/80
  // making assumption that -4.75 is -80 degrees
  public double getCurrentAltitudeAngle() {
    return (m_AltitudeEncoder.getPosition() / 4.75 * 80);
  }

  // Set a variable speed to move to a position
  public void setAltitude(double speed) {
    m_AltitudeMotor.set(speed * AltitudeConstants.kMaxAltitudeSpeedMetersPerSecond);
  }

  // Maintain position
  public void keepPosition(double position) {
    m_AltitudePIDController.setReference(position, ControlType.kPosition);
    SmartDashboard.putNumber("Altitude Desired position", position);
  }
}
