// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robot.Constants.ArmConstants;
import static frc.robot.Constants.ElevatorConstants;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;

public class PrepareDropoff extends SequentialCommandGroup {
    public PrepareDropoff(
            Arm m_arm,
            Elevator m_elevator) {
        addCommands(
                new ParallelCommandGroup(
                        new SetElevator(ElevatorConstants.kElevatorAltitudeDropOffPositionAngle, m_elevator),
                        new SetArm(ArmConstants.kArmPositionMidDropOff, m_arm)),
                new KeepAltitude(m_elevator.getCurrentAltitude(), m_elevator));
    }
}
