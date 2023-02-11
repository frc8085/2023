// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  // Elevator motors
  private final CANSparkMax m_ElevatorMotor = new CANSparkMax(ElevatorConstants.kElevatorMotorPort,
      MotorType.kBrushless);
  private final CANSparkMax m_ElevatorArmMotor = new CANSparkMax(ElevatorConstants.kElevatorArmMotorPort,
      MotorType.kBrushless);

  public Elevator() {
    m_ElevatorMotor.setOpenLoopRampRate(ElevatorConstants.kElevatorRampRate);
    m_ElevatorArmMotor.setOpenLoopRampRate(ElevatorConstants.kArmRampRate);
  }

  /** ELEVATOR ARM **/
  // Run the elevator arm motor forward
  public void extendElevatorArm() {
    m_ElevatorArmMotor.set(ElevatorConstants.kElevatorArmSpeed);
  }

  // Run the elevator arm motor in reverse
  public void retractElevatorArm() {
    m_ElevatorArmMotor.set(-ElevatorConstants.kElevatorArmSpeed);
  }

  // Stop the elevator arm
  public void stopArm() {
    m_ElevatorArmMotor.set(0);
  }

  /** ELEVATOR ALTITUDE **/
  // Run the elevator motor forward
  public void raiseElevator() {
    m_ElevatorMotor.set(-ElevatorConstants.kElevatorSpeed);
  }

  // Run the elevator motor in reverse
  public void lowerElevator() {
    m_ElevatorMotor.set(ElevatorConstants.kElevatorSpeed / 2);
  }

  // Stop the elevator the elevator
  public void stopElevator() {
    m_ElevatorMotor.set(0);
  }

  // Stop both the elevator and arm
  public void stop() {
    stopArm();
    stopElevator();
  }

}
