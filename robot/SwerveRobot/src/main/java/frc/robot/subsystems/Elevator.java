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
  private final CANSparkMax m_ArmMotor = new CANSparkMax(ElevatorConstants.kElevatorArmMotorPort,
      MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder m_ElevatorEncoder = m_ElevatorMotor.getEncoder();
  private final RelativeEncoder m_ArmEncoder = m_ArmMotor.getEncoder();

  // Limit Switches
  private SparkMaxLimitSwitch m_ArmExtensionLimit;
  private SparkMaxLimitSwitch m_ArmRetractionLimit;
  private SparkMaxLimitSwitch m_ElevatorTopLimit;
  private SparkMaxLimitSwitch m_ElevatorBottomLimit;

  public Elevator() {
    m_ElevatorMotor.setIdleMode(IdleMode.kBrake);
    m_ElevatorMotor.setSmartCurrentLimit(SubsystemMotorConstants.kMotorCurrentLimit);

    m_ElevatorMotor.setOpenLoopRampRate(ElevatorConstants.kElevatorRampRate);
    m_ArmMotor.setOpenLoopRampRate(ElevatorConstants.kArmRampRate);

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
    m_ElevatorTopLimit = m_ElevatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_ElevatorBottomLimit = m_ElevatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_ElevatorMotor.burnFlash();

  }

  /** The log method puts interesting information to the SmartDashboard. */
  public void log() {
    SmartDashboard.putNumber("ELEVATOR Raw encoder read", m_ElevatorEncoder.getPosition());
    SmartDashboard.putNumber("ARM Raw encoder read", m_ArmEncoder.getPosition());
    SmartDashboard.putBoolean("Arm Fully Extended", m_ArmExtensionLimit.isPressed());
    SmartDashboard.putBoolean("Arm Fully Retracted", m_ArmRetractionLimit.isPressed());
    SmartDashboard.putBoolean("Elevator at Top Position", m_ElevatorTopLimit.isPressed());
    SmartDashboard.putBoolean("Elevator at Bottom Position", m_ElevatorBottomLimit.isPressed());
  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();

    resetArmEncoderAtRetractionLimit();
    resetElevatorEncoderAtTopLimit();

    atElevatorAltitudeIntakePosition();

  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void reset() {
    m_ArmEncoder.setPosition(0);
    m_ElevatorEncoder.setPosition(0);
  }

  /** ELEVATOR ARM **/
  // Run the elevator arm motor forward
  public void extendElevatorArm() {
    m_ArmMotor.set(ElevatorConstants.kElevatorArmSpeed);
  }

  // Run the elevator arm motor in reverse
  public void retractElevatorArm() {
    m_ArmMotor.set(-ElevatorConstants.kElevatorArmSpeed);
  }

  // Stop the elevator arm
  public void stopArm() {
    m_ArmMotor.set(0);
  }

  // Determine if Limit Switches are hit
  public boolean isArmRetractionLimitHit() {
    return m_ArmRetractionLimit.isPressed() == true;
  }

  public boolean isArmExtensionLimitHit() {
    return m_ArmExtensionLimit.isPressed() == true;
  }

  // Reset the Arm Encoder when the Retraction Limit is pressed

  public void resetArmEncoderAtRetractionLimit() {
    if (isArmRetractionLimitHit()) {
      m_ArmEncoder.setPosition(0);
    }
  }

  // alternate way of writing the above statement
  // public void resetArmEncoderAtRetractionLimit() {
  // isArmRetractionLimitHit() && m_ArmEncoder.setPosition(0);
  // }

  /** ELEVATOR ALTITUDE **/
  // Run the elevator motor forward
  public void raiseElevator() {
    m_ElevatorMotor.set(ElevatorConstants.kElevatorSpeed);
  }

  // Run the elevator motor in reverse
  public void lowerElevator() {
    m_ElevatorMotor.set(-ElevatorConstants.kElevatorSpeed / 2);
  }

  // Stop the elevator hack
  public void stopElevator() {
    m_ElevatorMotor.set(ElevatorConstants.kElevatorStopSpeed);
  }

  public boolean isElevatorTopLimitHit() {
    return m_ElevatorTopLimit.isPressed() == true;
  }

  public boolean isElevatorBottomLimitHit() {
    return m_ElevatorBottomLimit.isPressed() == true;
  }

  public void resetElevatorEncoderAtTopLimit() {
    if (isElevatorTopLimitHit()) {
      m_ElevatorEncoder.setPosition(0);
    }
  };

  // Stop the elevator
  // public void stopElevator() {
  // m_ElevatorMotor.set(0);
  // }

  // Stop both the elevator and arm
  public void stop() {
    stopArm();
    stopElevator();
  }

  // Returns the current altitude of the elevator
  public double getCurrentAltitude() {
    return m_ElevatorEncoder.getPosition();
  }

  // Maintain the altitude
  public void maintain(double altitude) {
    m_ElevatorMotor.set(altitude);
  }

  // Determine if Altitude is at Intake Position
  public boolean atElevatorAltitudeIntakePosition() {
    double altitudeCurrentPosition = m_ElevatorEncoder.getPosition();
    double altitudeTolerance = ElevatorConstants.kAltitudePositionTolerance;

    double setpoint = ElevatorConstants.kElevatorAltitudeIntakePosition;
    double minLimit = setpoint - altitudeTolerance;
    double maxLimit = setpoint + altitudeTolerance;

    boolean withinLimits = isElevatorBottomLimitHit() ||
        (altitudeCurrentPosition >= minLimit
            && altitudeCurrentPosition <= maxLimit);
    return withinLimits;
  }

  // Determine if Altitude must be lowered or raised for intake position

  public boolean isElevatorAboveIntakePosition() {
    double altitudeCurrentPosition = m_ElevatorEncoder.getPosition();
    double setpoint = ElevatorConstants.kElevatorAltitudeIntakePosition;

    boolean abovePosition = (altitudeCurrentPosition > setpoint);
    return abovePosition;
  }

  // Determine if Altitude is at DropOff Position
  public boolean atElevatorAltitudeDropOffPosition() {
    double altitudeCurrentPosition = m_ElevatorEncoder.getPosition();
    double altitudeTolerance = ElevatorConstants.kAltitudePositionTolerance;
    double setpoint = ElevatorConstants.kElevatorAltitudeDropOffPosition;
    double minLimit = setpoint - altitudeTolerance;
    double maxLimit = setpoint + altitudeTolerance;

    boolean withinLimits = (altitudeCurrentPosition >= minLimit
        && altitudeCurrentPosition <= maxLimit);
    return withinLimits;

  }

  // Determine if Altitude is at Travel Position
  public boolean atElevatorAltitudeTravelPosition() {
    double altitudeCurrentPosition = m_ElevatorEncoder.getPosition();
    double altitudeTolerance = ElevatorConstants.kAltitudePositionTolerance;
    double setpoint = ElevatorConstants.kElevatorAltitudeTravelPosition;
    double minLimit = setpoint - altitudeTolerance;
    double maxLimit = setpoint + altitudeTolerance;

    boolean withinLimits = isElevatorTopLimitHit() ||
        (altitudeCurrentPosition >= minLimit
            && altitudeCurrentPosition <= maxLimit);
    return withinLimits;

  }

  // Move Altitude to Intake Position
  public void elevatorAltitudeIntakePosition() {
    if (atElevatorAltitudeIntakePosition()) {
      stopElevator();
    } else {
      if (isElevatorAboveIntakePosition()) {
        lowerElevator();
      } else {
        raiseElevator();
      }
    }
  }

  // Maintain the altitude
  // public void maintain(double altitude) {
  // m_ElevatorMotor.set(altitude);
  // }

}