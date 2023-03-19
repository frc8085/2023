// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AltitudeConstants;
import frc.robot.Constants.ExtensionConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/** An example command that uses an example subsystem. */
public class AutoSidekick extends SequentialCommandGroup {
  public AutoSidekick(
      DriveSubsystem m_drive,
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    addCommands(

        // 1. score
        new ParallelCommandGroup(
            new Extend(m_extension, ExtensionConstants.kExtensionPositionHighDropOff),
            new RaiseLower(m_altitude, AltitudeConstants.kAltitudeHighDropOffFinalPosition)),
        new Extend(m_extension,
            ExtensionConstants.kExtensionPositionHighDropOff - ExtensionConstants.kExtensionConeRetractDistance),
        // new ScoreHighCone(m_altitude, m_extension, m_intake)
        new InstantCommand(() -> m_intake.ejectCone()),
        new WaitCommand(IntakeConstants.kEjectWaitTime),
        new InstantCommand(() -> m_intake.stopIntake()),

        new ParallelCommandGroup(
            new RaiseLower(m_altitude, AltitudeConstants.kAltitudeTravelPosition),
            new WaitUntilCommand(() -> m_altitude.AltitudeIsInScoringPosition()),
            new Extend(m_extension, ExtensionConstants.kExtensionPositionFullyRetracted))

    //

    );

    // 2. drive back 3m & turn 180
    // 3. move tontake down
    // 4. drive forward 2 meters
    // 5. travel position
    // 6. drive to 1 turn 180
    // 7. shoot

  }
}