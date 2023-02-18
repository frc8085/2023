// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  // Arm motors
  private final CANSparkMax m_armMotor = new CANSparkMax(ArmConstants.kArmMotorPort,
      MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder m_armEncoder = m_armMotor.getEncoder();

  // Limit Switches
  private SparkMaxLimitSwitch m_armExtensionLimit;
  private SparkMaxLimitSwitch m_armRetractionLimit;

  public boolean ArmIsInTravelPosition() {
    return isArmRetractionLimitHit();
  }

  public Arm() {
    m_armMotor.setOpenLoopRampRate(ArmConstants.kArmRampRate);

    /**
     * A SparkMaxLimitSwitch object is constructed using the getForwardLimitSwitch()
     * or
     * on which direction you would like to limit
     * 
     * Limit switches can be configured to one of two polarities:
     * com.revrobotics.SparkMaxLimitSwitch.SparkMaxLimitSwitch.Type.kNormallyOpen
     * com.revrobotics.SparkMaxLimitSwitch.SparkMaxLimitSwitch.Type.kNormallyClosed
     */
    m_armExtensionLimit = m_armMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_armRetractionLimit = m_armMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    // m_Ar,Motor.burnFlash();

  }

  /** The log method puts interesting information to the SmartDashboard. */
  public void log() {
    SmartDashboard.putNumber("ARM Raw encoder read", m_armEncoder.getPosition());
    SmartDashboard.putBoolean("Arm Fully Extended", m_armExtensionLimit.isPressed());
    SmartDashboard.putBoolean("Arm Fully Retracted", m_armRetractionLimit.isPressed());
    SmartDashboard.putBoolean("Arm Travel Position", ArmIsInTravelPosition());
  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();
    resetArmEncoderAtRetractionLimit();
    ArmIsInTravelPosition();

  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void reset() {
    m_armEncoder.setPosition(0);
  }

  /** ELEVATOR ARM **/
  // Run the elevator arm motor forward
  public void extendArm() {
    m_armMotor.set(ArmConstants.kArmSpeed);
  }

  // Run the elevator arm motor in reverse
  public void retractArm() {
    m_armMotor.set(-ArmConstants.kArmSpeed);
  }

  // Stop the elevator arm
  public void stopArm() {
    m_armMotor.set(0);
  }

  // Reset the Arm Encoder when the Retraction Limit is pressed

  public boolean isArmRetractionLimitHit() {
    return m_armRetractionLimit.isPressed() == true;
  }

  public void resetArmEncoderAtRetractionLimit() {
    if (isArmRetractionLimitHit()) {
      m_armEncoder.setPosition(0);
    }
  }

  // alternate way of writing the above statement
  // public void resetArmEncoderAtRetractionLimit() {
  // isArmRetractionLimitHit() && m_armEncoder.setPosition(0);
  // }

  // Returns the current position of the arm
  public double getCurrentArmPosition() {
    return (m_armEncoder.getPosition());
  }

  // Set a variable speed
  public void setArm(double speed) {
    m_armMotor.set(speed * ArmConstants.kMaxArmSpeedMetersPerSecond);
    SmartDashboard.putNumber("Arm PID Speed Output", speed);
  }

}
