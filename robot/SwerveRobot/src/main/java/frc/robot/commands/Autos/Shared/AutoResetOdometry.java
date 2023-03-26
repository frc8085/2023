// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Shared;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

/** Reset the heading and odometry */
public class AutoResetOdometry extends SequentialCommandGroup {
    public AutoResetOdometry(
            DriveSubsystem m_drive) {
        addCommands(
        // new RunCommand(() -> m_drive.zeroHeading()).withTimeout(0.5),
        // new RunCommand(() -> m_drive.resetOdometry(new Pose2d())).withTimeout((0.5))
        );
    }
}
