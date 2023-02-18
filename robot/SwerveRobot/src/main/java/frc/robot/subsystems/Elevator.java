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
import static frc.robot.Constants.ElevatorConstants;
import static frc.robot.Constants.SubsystemMotorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  // Elevator motors
  private final CANSparkMax m_ElevatorMotor = new CANSparkMax(ElevatorConstants.kElevatorMotorPort,
      MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder m_ElevatorEncoder = m_ElevatorMotor.getEncoder();

  // Limit Switches
  private SparkMaxLimitSwitch m_ElevatorTopLimit;
  private SparkMaxLimitSwitch m_ElevatorBottomLimit;

  public boolean ElevatorIsInTravelPosition() {
    return isElevatorTopLimitHit();
  }

  public Elevator() {
    m_ElevatorMotor.setIdleMode(IdleMode.kBrake);
    m_ElevatorMotor.setSmartCurrentLimit(SubsystemMotorConstants.kMotorCurrentLimit);

    m_ElevatorMotor.setOpenLoopRampRate(ElevatorConstants.kElevatorRampRate);

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
    m_ElevatorTopLimit = m_ElevatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_ElevatorBottomLimit = m_ElevatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_ElevatorMotor.burnFlash();

  }

  /** The log method puts interesting information to the SmartDashboard. */
  public void log() {
    SmartDashboard.putNumber("ELEVATOR Raw encoder read", m_ElevatorEncoder.getPosition());
    SmartDashboard.putBoolean("Elevator at Top Position", m_ElevatorTopLimit.isPressed());
    SmartDashboard.putBoolean("Elevator at Bottom Position", m_ElevatorBottomLimit.isPressed());
    SmartDashboard.putBoolean("Elevator is in Travel Position", ElevatorIsInTravelPosition());

    SmartDashboard.putNumber("Current altitude", getCurrentAltitude());
    SmartDashboard.putNumber("Current altitude angle", getCurrentAltitudeAngle());

  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();

    resetElevatorEncoderAtTopLimit();
    ElevatorIsInTravelPosition();

  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void reset() {
    m_ElevatorEncoder.setPosition(0);
  }

  // Reset the Arm Encoder when the Retraction Limit is pressed

  public boolean isElevatorTopLimitHit() {
    return m_ElevatorTopLimit.isPressed() == true;
  }

  // alternate way of writing the above statement
  // public void resetArmEncoderAtRetractionLimit() {
  // isArmRetractionLimitHit() && m_ArmEncoder.setPosition(0);
  // }

  public void resetElevatorEncoderAtTopLimit() {
    if (isElevatorTopLimitHit()) {
      m_ElevatorEncoder.setPosition(0);
    }
  };

  /** ELEVATOR ALTITUDE **/
  // Run the elevator motor forward
  public void raiseElevator() {
    m_ElevatorMotor.set(ElevatorConstants.kElevatorSpeed);
  }

  // Run the elevator motor in reverse
  public void lowerElevator() {
    m_ElevatorMotor.set(-ElevatorConstants.kElevatorSpeed);
  }

  // Stop the elevator hack
  public void stopElevator() {
    m_ElevatorMotor.set(ElevatorConstants.kElevatorStopSpeed);
  }

  // Stop the elevator
  // public void stopElevator() {
  // m_ElevatorMotor.set(0);
  // }

  // Returns the current altitude of the elevator
  public double getCurrentAltitude() {
    return m_ElevatorEncoder.getPosition();
  }

  // Returns the current altitude of the elevator in degrees
  public double getCurrentAltitudeAngle() {
    return (m_ElevatorEncoder.getPosition() / 4.75 * 80);
  }

  // Set a variable speed
  public void setElevator(double speed) {
    m_ElevatorMotor.set(speed * ElevatorConstants.kMaxElevatorAltitudeSpeedMetersPerSecond);
    SmartDashboard.putNumber("PID Speed Output", speed);
  }

  // Move the elevator altitude to travel position
  public void MoveElevatorToTravelPosition() {
    if (m_ElevatorEncoder.getPosition() < ElevatorConstants.kElevatorAltitudeTravelPosition) {
      raiseElevator();
    } else {
      stopElevator();
    }
  }

  // change altitude encoder readings to degrees angle*4.75/80
  // making assumption that -4.75 is -80 degrees
  public double getElevatorAngle() {
    return m_ElevatorEncoder.getPosition() / 4.75 * 80;

  }
}
