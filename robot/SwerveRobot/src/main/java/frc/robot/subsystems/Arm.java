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
  private final CANSparkMax m_ArmMotor = new CANSparkMax(ArmConstants.kElevatorArmMotorPort,
      MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder m_ArmEncoder = m_ArmMotor.getEncoder();

  // Limit Switches
  private SparkMaxLimitSwitch m_ArmExtensionLimit;
  private SparkMaxLimitSwitch m_ArmRetractionLimit;

  // Assume robot is not in travel position to start
  private boolean TravelPosition = false;

  // Create a condition for when elevator is in travel mode
  public boolean TravelPosition() {
    return TravelPosition;
  }

  public void ElevatorIsInTravelPosition() {
    if (isArmRetractionLimitHit()) {
      TravelPosition = true;
    }
  }

  public Arm() {
    m_ArmMotor.setOpenLoopRampRate(ArmConstants.kArmRampRate);

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
    m_ArmExtensionLimit = m_ArmMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_ArmRetractionLimit = m_ArmMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    // m_Ar,Motor.burnFlash();

  }

  /** The log method puts interesting information to the SmartDashboard. */
  public void log() {
    SmartDashboard.putNumber("ARM Raw encoder read", m_ArmEncoder.getPosition());
    SmartDashboard.putBoolean("Arm Fully Extended", m_ArmExtensionLimit.isPressed());
    SmartDashboard.putBoolean("Arm Fully Retracted", m_ArmRetractionLimit.isPressed());
  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();
    resetArmEncoderAtRetractionLimit();

  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void reset() {
    m_ArmEncoder.setPosition(0);
  }

  /** ELEVATOR ARM **/
  // Run the elevator arm motor forward
  public void extendElevatorArm() {
    m_ArmMotor.set(ArmConstants.kElevatorArmSpeed);
  }

  // Run the elevator arm motor in reverse
  public void retractElevatorArm() {
    m_ArmMotor.set(-ArmConstants.kElevatorArmSpeed);
  }

  // Stop the elevator arm
  public void stopArm() {
    m_ArmMotor.set(0);
  }

  // Reset the Arm Encoder when the Retraction Limit is pressed

  public boolean isArmRetractionLimitHit() {
    return m_ArmRetractionLimit.isPressed() == true;
  }

  public void resetArmEncoderAtRetractionLimit() {
    if (isArmRetractionLimitHit()) {
      m_ArmEncoder.setPosition(0);
    }
  }

  // alternate way of writing the above statement
  // public void resetArmEncoderAtRetractionLimit() {
  // isArmRetractionLimitHit() && m_ArmEncoder.setPosition(0);
  // }

  public void MoveArmToTravelPosition() {
    // Retract the elevator arm to travel position
    if (m_ArmEncoder.getPosition() > ArmConstants.kArmPositionFullyRetracted) {
      retractElevatorArm();
    } else {
      stopArm();
    }
  }

  // Set elevator arm to intake position - this won't work because it needs
  // tolerance
  public void MoveArmToIntakePosition() {
    if (m_ArmEncoder.getPosition() > ArmConstants.kArmPositionIntakeOut) {
      retractElevatorArm();
    } else {
      if (m_ArmEncoder.getPosition() < ArmConstants.kArmPositionIntakeOut) {
        extendElevatorArm();
      } else {
        stopArm();
      }
    }
  }
}
