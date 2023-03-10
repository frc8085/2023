// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AltitudeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoRotateDegrees;
import frc.robot.commands.Autos;
import frc.robot.commands.MoveToMidConeDropOff;
import frc.robot.commands.MoveToHighConeDropOff;
import frc.robot.commands.MoveToTravel;
import frc.robot.commands.MoveToTravelAfterIntake;
import frc.robot.commands.MoveToTravelAfterScoring;
import frc.robot.commands.ResetPositionToStart;
import frc.robot.commands.ScoreBasedOnPosition;
import frc.robot.commands.ScoreHighCube;
import frc.robot.commands.IntakeCargo;
import frc.robot.commands.IntakeCargoFromDoubleSubstation;
import frc.robot.commands.IntakeCargoFromSingleSubstation;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.Altitude;
import edu.wpi.first.wpilibj.DriverStation;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Add Auto Selection chooser to Dashboard
  protected SendableChooser<Command> autoSelection = new SendableChooser<>();

  // The robot's subsystems
  private final LEDs m_leds = new LEDs();
  private final Intake m_intake = new Intake(m_leds);
  private final Extension m_extension = new Extension();
  private final Altitude m_altitude = new Altitude(m_extension);
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_altitude, m_extension);

  // The controllers
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    // Zero heading and reset odometry when we start
    m_robotDrive.zeroHeading();
    m_robotDrive.resetOdometry(new Pose2d());
    m_intake.reset();

    configureAuto();

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // Right Bumper sets a reduced max speed limit
        // Right Trigger controls speed
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(() -> m_robotDrive.drive(
            m_driverController.getRightBumper(),
            m_driverController.getRightTriggerAxis(),
            // From Rev for SlewRate Limiter
            Math.max(0.0, (Math.abs(m_driverController.getLeftY())
                - OIConstants.kDriveDeadband)
                / (1.0 - OIConstants.kDriveDeadband))
                * Math.signum(m_driverController
                    .getLeftY()),
            Math.max(0.0, (Math.abs(m_driverController.getLeftX())
                - OIConstants.kDriveDeadband)
                / (1.0 - OIConstants.kDriveDeadband))
                * Math.signum(m_driverController
                    .getLeftX()),
            Math.max(0.0, (Math.abs(m_driverController
                .getRightX())
                - OIConstants.kDriveDeadband)
                / (1.0 - OIConstants.kDriveDeadband))
                * Math.signum(-m_driverController
                    .getRightX()),
            true,
            // rateLimit is true if rightBumper is not pressed,
            // and we are within safe limits
            // false if it is
            !m_driverController.getRightBumper()
                && m_robotDrive.isWithinSafeDrivingLimits()

        ),
            m_robotDrive));
  }

  private void configureAuto() {

    autoSelection.setDefaultOption("(18pt) TEST: Score High, Balance",
        Autos.scoreHighAndBalanceByDistance(m_robotDrive, m_altitude, m_extension, m_intake));
    autoSelection.addOption("PRACTICE turn", new AutoRotateDegrees(m_robotDrive, 180));
    autoSelection.addOption("(18pt) RELIABLE: Score High, Balance",
        Autos.scoreHighAndBalance(m_robotDrive, m_altitude, m_extension, m_intake));
    autoSelection.addOption("(9pt) SIDEKICK: Score High, Leave, Pickup",
        Autos.scoreHighLeaveAndPickup(m_robotDrive, m_altitude, m_extension, m_intake));
    autoSelection.addOption("(15pt) IDEAL SIDEKICK: Score High, Leave, Pickup, Return and Score Again",
        Autos.scoreHighLeavePickupReturnandScore(m_robotDrive, m_altitude, m_extension, m_intake));
    autoSelection.addOption("(21pt) IDEAL: Score High, Leave, Pickup, Balance",
        Autos.scoreHighLeavePickupAndBalance(m_robotDrive, m_altitude, m_extension, m_intake));
    autoSelection.addOption("(12pt) MINIMAL: BalanceByDistance", Autos.balanceByDistance(m_robotDrive));
    autoSelection.addOption("(6pt) MINIMAL: Score High",
        Autos.scoreHigh(m_robotDrive, m_altitude, m_extension, m_intake));

    // Put the chooser on the dashboard
    SmartDashboard.putData("Auto Routine", autoSelection);
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    /** DRIVE LOCK **/
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .toggleOnTrue(new RunCommand(() -> m_robotDrive.lock(), m_robotDrive));

    new JoystickButton(m_driverController, Button.kY.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    /** OPERATOR COMMANDS **/
    final Trigger startButton = m_operatorController.start();
    final Trigger intakeButton = m_operatorController.rightTrigger();
    final Trigger manualIntakeButton = m_operatorController.rightBumper();
    final Trigger ejectButton = m_operatorController.leftTrigger();
    final Trigger highCubeEjectButton = m_operatorController.leftBumper();
    // final Trigger midCubeEjectButton = m_operatorController.;
    final Trigger setDoubleSubstationButton = m_operatorController.povUp();
    final Trigger setSingleSubstationButton = m_operatorController.povDown();
    final Trigger ExtendButton = m_operatorController.axisLessThan(5, -.25);
    final Trigger RetractButton = m_operatorController.axisGreaterThan(5, .25);
    final Trigger RaiseButton = m_operatorController.axisLessThan(1, -.25);
    final Trigger LowerButton = m_operatorController.axisGreaterThan(1, .25);

    startButton.onTrue(new ResetPositionToStart(m_altitude, m_extension));

    // Intake button will run the intake then hold game piece when released
    intakeButton.whileTrue(new IntakeCargo(m_altitude, m_extension, m_intake))
        .onFalse(new ParallelCommandGroup(
            new InstantCommand(() -> m_intake.holdCargo()),
            new MoveToTravelAfterIntake(m_extension, m_altitude)));

    manualIntakeButton.whileTrue(
        new InstantCommand(() -> m_intake.intakeCube()))
        .onFalse(new InstantCommand(() -> m_intake.holdCargo()));

    setDoubleSubstationButton
        .whileTrue(new IntakeCargoFromDoubleSubstation(m_altitude, m_extension, m_intake))
        .onFalse(new ParallelCommandGroup(
            new InstantCommand(() -> m_intake.holdCargo()),
            new MoveToTravelAfterScoring(m_extension, m_altitude)));

    setSingleSubstationButton
        .whileTrue(new IntakeCargoFromSingleSubstation(m_altitude, m_extension, m_intake))
        .onFalse(new ParallelCommandGroup(
            new InstantCommand(() -> m_intake.holdCargo()),
            new MoveToTravelAfterScoring(m_extension, m_altitude)));

    ejectButton.onTrue(new ScoreBasedOnPosition(m_altitude, m_extension,
        m_intake));

    highCubeEjectButton.onTrue(new ScoreHighCube(m_altitude, m_extension, m_intake));

    // midCubeEjectButton.onTrue(new ScoreMidCube(m_altitude, m_extension,
    // m_intake));

    ExtendButton.whileTrue(new InstantCommand(m_extension::extendExtension, m_extension))
        .onFalse(new InstantCommand(
            () -> m_extension.keepPosition(
                m_extension.getCurrentExtensionPosition())));

    RetractButton.whileTrue(new InstantCommand(m_extension::retractExtension, m_extension))
        .onFalse(new InstantCommand(
            () -> m_extension.keepPosition(
                m_extension.getCurrentExtensionPosition())));

    RaiseButton
        .whileTrue(new InstantCommand(m_altitude::raiseAltitude, m_altitude))
        .onFalse(new InstantCommand(
            () -> m_altitude.keepPosition(m_altitude.getCurrentAltitude())));

    LowerButton
        .whileTrue(new InstantCommand(m_altitude::lowerAltitude, m_altitude))
        .onFalse(new InstantCommand(
            () -> m_altitude.keepPosition(m_altitude.getCurrentAltitude())));

    /** PRESET POSITIONS **/
    final Trigger prepareHighDropOffButton = m_operatorController.b();
    final Trigger prepareMidDropOffButton = m_operatorController.x();
    final Trigger prepareTravelButton = m_operatorController.y();
    // final Trigger prepareIntakeButton = m_operatorController.a();
    // final Trigger prepareShelfPickupButton = m_operatorController.start();

    prepareMidDropOffButton.onTrue(new MoveToMidConeDropOff(m_extension, m_altitude))
        .onFalse(new SequentialCommandGroup(
            new WaitUntilCommand(
                () -> m_extension.ExtensionIsInMidScoringPosition()),
            new InstantCommand(
                () -> m_altitude.keepPosition(
                    AltitudeConstants.kAltitudeMidDropOffPosition))));
    prepareHighDropOffButton.onTrue(new MoveToHighConeDropOff(m_extension, m_altitude))
        .onFalse(new SequentialCommandGroup(
            new WaitUntilCommand(
                () -> m_extension.ExtensionIsInHighScoringPosition()),
            new InstantCommand(
                () -> m_altitude.keepPosition(
                    AltitudeConstants.kAltitudeHighDropOffPosition))));
    prepareTravelButton.onTrue(new MoveToTravel(m_extension, m_altitude));
    // prepareIntakeButton.onTrue(new PrepareIntake(m_extension, m_altitude));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // return autoSelection.getSelected();
    // return Autos.balanceByDistance(m_robotDrive);
    // return new AutoRotateDegrees(m_robotDrive, 180);
    return Autos.scoreHighLeaveAndPickup(m_robotDrive, m_altitude, m_extension,
        m_intake);
  }

}
