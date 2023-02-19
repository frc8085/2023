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
import static frc.robot.Constants.ElevatorConstants;
import static frc.robot.Constants.SubsystemMotorConstants;

import javax.swing.text.Position;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private boolean isKeepingAltitude = false;

  // Elevator motors
  private final CANSparkMax m_elevatorMotor = new CANSparkMax(ElevatorConstants.kElevatorMotorPort,
      MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder m_elevatorEncoder = m_elevatorMotor.getEncoder();

  // Limit Switches
  private SparkMaxLimitSwitch m_elevatorTopLimit;
  private SparkMaxLimitSwitch m_elevatorBottomLimit;

  // PID
  private SparkMaxPIDController m_elevatorPIDController = m_elevatorMotor.getPIDController();
  static double kP = 1;
  static double kI = 0;
  static double kD = 0;

  public boolean ElevatorIsInTravelPosition() {
    return isElevatorTopLimitHit();
  }

  public boolean IsElevatorKeepingAltitude() {
    return isKeepingAltitude;
  }

  public void startKeepingAltitude() {
    isKeepingAltitude = true;
  }

  public void stopKeepingAltitude() {
    isKeepingAltitude = false;
  }

  public Elevator() {
    m_elevatorMotor.setIdleMode(IdleMode.kBrake);
    m_elevatorMotor.setSmartCurrentLimit(SubsystemMotorConstants.kMotorCurrentLimit);
    m_elevatorMotor.setOpenLoopRampRate(ElevatorConstants.kElevatorRampRate);

    m_elevatorPIDController.setFeedbackDevice(m_elevatorEncoder);
    m_elevatorPIDController.setP(kP, 0);
    m_elevatorPIDController.setI(kI, 0);
    m_elevatorPIDController.setD(kD, 0);
    m_elevatorPIDController.setOutputRange(-0.5, 0.5);

    // TODO. What should these values be?
    m_elevatorPIDController.setSmartMotionMaxAccel(0.5, 0);
    m_elevatorPIDController.setSmartMotionMaxVelocity(0.5, 0);

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
    m_elevatorTopLimit = m_elevatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_elevatorBottomLimit = m_elevatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_elevatorMotor.burnFlash();

  }

  /** The log method puts interesting information to the SmartDashboard. */
  public void log() {
    SmartDashboard.putNumber("ELEVATOR Raw encoder read", m_elevatorEncoder.getPosition());
    SmartDashboard.putBoolean("Elevator at Top Position", m_elevatorTopLimit.isPressed());
    SmartDashboard.putBoolean("Elevator at Bottom Position", m_elevatorBottomLimit.isPressed());
    SmartDashboard.putBoolean("Elevator is in Travel Position", ElevatorIsInTravelPosition());

    SmartDashboard.putNumber("Current altitude", getCurrentAltitude());

    SmartDashboard.putBoolean("Is keeping altitude", isKeepingAltitude);

  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();

    resetElevatorEncoderAtTopLimit();
    ElevatorIsInTravelPosition();

  }

  /** Resets the elevator encoder to currently read a position of 0. */
  public void reset() {
    m_elevatorEncoder.setPosition(0);
  }

  // Reset the Arm Encoder when the Retraction Limit is pressed

  public boolean isElevatorTopLimitHit() {
    return m_elevatorTopLimit.isPressed() == true;
  }

  public void resetElevatorEncoderAtTopLimit() {
    if (isElevatorTopLimitHit()) {
      m_elevatorEncoder.setPosition(-0.1);
    }
  };

  /** ELEVATOR ALTITUDE **/
  // Run the elevator motor forward
  public void raiseElevator() {
    m_elevatorMotor.set(ElevatorConstants.kElevatorSpeed);
  }

  // Run the elevator motor in reverse
  public void lowerElevator() {
    m_elevatorMotor.set(-ElevatorConstants.kElevatorSpeed);
  }

  // Stop the elevator hack
  // public void stopElevator() {
  // m_elevatorMotor.set(ElevatorConstants.kElevatorStopSpeed);
  // }

  // Stop the elevator
  public void stopElevator() {
    m_elevatorMotor.set(0);
  }

  // Returns the current altitude of the elevator
  public double getCurrentAltitude() {
    return m_elevatorEncoder.getPosition();
  }

  // Returns the current altitude of the elevator in degrees
  public double getCurrentAltitudeAngle() {
    return (m_elevatorEncoder.getPosition() / 4.75 * 80);
  }

  // Set a variable speed to move to a position
  public void setElevator(double speed) {
    stopKeepingAltitude();
    m_elevatorMotor.set(speed * ElevatorConstants.kMaxElevatorAltitudeSpeedMetersPerSecond);
    SmartDashboard.putNumber("PID Speed Output", speed);
  }

  // Maintain position
  public void keepPosition(double position) {
    m_elevatorPIDController.setReference(position, ControlType.kPosition);
    SmartDashboard.putNumber("Desired position", position);
    SmartDashboard.putNumber("Encoder position", m_elevatorEncoder.getPosition());
  }

  // change altitude encoder readings to degrees angle*4.75/80
  // making assumption that -4.75 is -80 degrees
  public double getElevatorAngle() {
    return m_elevatorEncoder.getPosition() / 4.75 * 80;

  }
}