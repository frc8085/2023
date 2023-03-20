// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class FaceIntakeToDirection {
    public enum Direction {
        INTAKE_DOWN, INTAKE_LEFT, INTAKE_UP, INTAKE_RIGHT
    }

    public static Command command(DriveSubsystem drive, Direction direction) {
        double absoluteDegrees;
        switch (direction) {
            case INTAKE_DOWN:
                absoluteDegrees = 0;
            case INTAKE_LEFT:
                absoluteDegrees = -90;
            case INTAKE_UP:
                absoluteDegrees = -180;
            case INTAKE_RIGHT:
                absoluteDegrees = +90;
            default:
                absoluteDegrees = 0;
        }

        return new AutoTurnToDegreeGyro(absoluteDegrees, drive, false);
    }
}
