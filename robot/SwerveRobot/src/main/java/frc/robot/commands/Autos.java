// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AltitudeConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public final class Autos {

  public static CommandBase initialize(DriveSubsystem m_drive, Altitude m_altitude, Extension m_extension) {
    return Commands.sequence(
        new ResetPositionToStart(m_altitude, m_extension),
        new InstantCommand(m_drive::zeroHeading));
  }

  public static CommandBase balance(DriveSubsystem m_drive) {
    return Commands.sequence(
        new AutoDriveToReachStation(m_drive),
        new AutoDriveToBalance(m_drive),
        new AutoFinalBalance(m_drive),
        new RunCommand(m_drive::lock, m_drive));
  }

  public static CommandBase balance2(DriveSubsystem m_drive) {
    return Commands.sequence(
        new AutoDriveBackwardsMeters(m_drive, 2.4, .4),
        new RunCommand(m_drive::lock, m_drive));
  }

  public static CommandBase balanceByDistance(DriveSubsystem m_drive) {
    return Commands.sequence(
        new AutoDriveBackwardsMeters(m_drive, 2, .4),
        new AutoDriveBackwardsMeters(m_drive, .65, .2),
        new AutoDriveToBalanceByDistance(m_drive),
        new AutoFinalBalance(m_drive),
        new RunCommand(m_drive::lock, m_drive));
  }

  public static CommandBase scoreHigh(DriveSubsystem m_drive, Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    return Commands.sequence(
        new MoveToHighConeDropOff(m_extension, m_altitude)
            .until(() -> m_extension.ExtensionIsInHighScoringPosition()),
        new WaitUntilCommand(
            () -> m_extension.ExtensionIsInHighScoringPosition()),
        new InstantCommand(
            () -> m_altitude.keepPosition(
                AltitudeConstants.kAltitudeHighDropOffPosition)),
        new WaitCommand(.5),
        new AutoScoreHighCone(m_altitude, m_extension, m_intake));
  }

  public static CommandBase scoreHigh2(DriveSubsystem m_drive, Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    return Commands.sequence(
        new MoveToHighConeDropOff(m_extension, m_altitude)
            .until(() -> m_extension.ExtensionIsInHighScoringPosition()),
        new WaitUntilCommand(
            () -> m_extension.ExtensionIsInHighScoringPosition()),
        new InstantCommand(
            () -> m_altitude.keepPosition(
                AltitudeConstants.kAltitudeHighDropOffPosition)),
        new WaitCommand(.5),
        new AutoScoreHighCone(m_altitude, m_extension, m_intake));
  }

  public static CommandBase intakeAndHold(Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    return Commands.sequence(
        new IntakeCargo(m_altitude, m_extension, m_intake).withTimeout(2),
        new InstantCommand(() -> m_intake.holdCargo()),
        new MoveToTravelAfterIntake(m_extension, m_altitude));
  }

  public static CommandBase scoreHighAndBalanceByDistance(DriveSubsystem m_drive, Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    return Commands.sequence(
        initialize(m_drive, m_altitude, m_extension),
        scoreHigh(m_drive, m_altitude, m_extension, m_intake),
        new ParallelCommandGroup(
            new SequentialCommandGroup(new WaitCommand(IntakeConstants.kEjectWaitTime),
                new InstantCommand(m_intake::stopIntake)),
            new MoveToTravelAfterScoring(m_extension, m_altitude),
            balanceByDistance(m_drive)));
  }

  public static CommandBase scoreHighAndBalance(DriveSubsystem m_drive, Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    return Commands.sequence(
        initialize(m_drive, m_altitude, m_extension),
        scoreHigh(m_drive, m_altitude, m_extension, m_intake),
        new ParallelCommandGroup(
            new MoveToTravelAfterScoring(m_extension, m_altitude),
            balance(m_drive)),
        new InstantCommand(m_intake::stopIntake));
  }

  public static CommandBase scoreHighAndLeave(DriveSubsystem m_drive, Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    return Commands.sequence(
        initialize(m_drive, m_altitude, m_extension),
        scoreHigh(m_drive, m_altitude, m_extension, m_intake),
        new ParallelCommandGroup(
            new MoveToTravelAfterScoring(m_extension, m_altitude),
            new AutoDriveBackwardsMeters(m_drive, 4.75, .4)),
        new InstantCommand(m_intake::stopIntake));
  }

  // still testing
  public static CommandBase scoreHighLeaveAndPickup(DriveSubsystem m_drive, Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    return Commands.sequence(
        initialize(m_drive, m_altitude, m_extension),
        scoreHigh(m_drive, m_altitude, m_extension, m_intake),
        new ParallelCommandGroup(
            new SequentialCommandGroup(new WaitCommand(IntakeConstants.kEjectWaitTime),
                new InstantCommand(m_intake::stopIntake)),
            new MoveToTravelAfterScoring(m_extension, m_altitude),
            new AutoDriveBackwardsMeters(m_drive, 2.25, .4)),
        new AutoRotateDegrees(m_drive, 180),
        new PrepareIntake(m_extension, m_altitude),
        new InstantCommand(m_intake::intakeCone),
        new WaitUntilCommand(() -> m_altitude.AltitudeIsInIntakePosition()),
        new AutoDriveBackwardsMeters(m_drive, 1, .4),
        new InstantCommand(() -> m_intake.holdCargo()),
        new MoveToTravelAfterIntake(m_extension, m_altitude));
  }

  public static CommandBase scoreHighLeavePickupReturnandScore(DriveSubsystem m_drive, Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    return Commands.sequence(
        initialize(m_drive, m_altitude, m_extension),
        scoreHigh(m_drive, m_altitude, m_extension, m_intake),
        new AutoDriveBackwardsMeters(m_drive, 5, .4),
        new AutoRotateDegrees(m_drive, 180),
        intakeAndHold(m_altitude, m_extension, m_intake),
        new AutoRotateDegrees(m_drive, 180),
        new AutoDriveForwardMeters(m_drive, 5),
        scoreHigh(m_drive, m_altitude, m_extension, m_intake));
  }

  public static CommandBase scoreHighLeavePickupAndBalance(DriveSubsystem m_drive, Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    return Commands.sequence(
        initialize(m_drive, m_altitude, m_extension),
        scoreHigh(m_drive, m_altitude, m_extension, m_intake),
        new AutoDriveBackwardsMeters(m_drive, 5, .4),
        new AutoRotateDegrees(m_drive, 180),
        intakeAndHold(m_altitude, m_extension, m_intake),
        balance(m_drive));
  }

  // still testing
  public static CommandBase newScoreHighAndBalance(DriveSubsystem m_drive, Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    return Commands.sequence(
        initialize(m_drive, m_altitude, m_extension),
        scoreHigh(m_drive, m_altitude, m_extension, m_intake),
        new ParallelCommandGroup(
            new MoveToTravelAfterScoring(m_extension, m_altitude),
            balance2(m_drive)),
        new InstantCommand(m_intake::stopIntake));
  }

  public static CommandBase ScoreLeaveAndDock(DriveSubsystem m_drive, Altitude m_altitude,
      Extension m_extension, Intake m_intake) {
    return Commands.sequence(
        initialize(m_drive, m_altitude, m_extension),
        scoreHigh(m_drive, m_altitude, m_extension, m_intake),
        new ParallelCommandGroup(
            new SequentialCommandGroup(new WaitCommand(IntakeConstants.kEjectWaitTime),
                new InstantCommand(m_intake::stopIntake)),
            new MoveToTravelAfterScoring(m_extension, m_altitude),
            new AutoDriveBackwardsMeters(m_drive, 4.75, .4),
            new AutoDriveForwardMeters(m_drive, 2.25),
            new AutoRotateDegrees(m_drive, 45),
            new RunCommand(m_drive::lock)));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
