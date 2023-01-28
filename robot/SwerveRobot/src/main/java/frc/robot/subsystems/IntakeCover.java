// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static frc.robot.Constants.IntakeCoverConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeCover extends SubsystemBase {
  private final DoubleSolenoid m_intakeCoverSolenoid = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM,
      kIntakeCoverSolenoidPorts[0],
      kIntakeCoverSolenoidPorts[1]);

  /** Puts the intake cover down */
  public void openIntake() {
    m_intakeCoverSolenoid.set(kForward);
  }

  /** Lifts the intake cover up */
  public void closeIntake() {
    m_intakeCoverSolenoid.set(kReverse);
  }

  public boolean isIntakeCoverDown() {
    return m_intakeCoverSolenoid.get() == kForward;
  }
}