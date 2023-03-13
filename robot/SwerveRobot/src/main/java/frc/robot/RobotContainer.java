// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AltitudeConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoDriveMeters;
import frc.robot.commands.AutoDriveMetersAndTurn;
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
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
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

  private final Intake m_intake = new Intake();
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
    // Instantiate LEDs
    LEDs.getInstance();

    DriverStation.silenceJoystickConnectionWarning(true);

    // Zero heading and reset odometry when we start
    // m_robotDrive.zeroHeading();
    // m_robotDrive.resetOdometry(new Pose2d());
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

    autoSelection.setDefaultOption("(21pt) MAIN CHARACTER: Score High, Leave, Dock",
        Autos.ScoreLeaveAndDock(m_robotDrive, m_altitude, m_extension, m_intake));
    autoSelection.addOption("(21pt) SUPERHERO: Score High, Leave, Balance",
        Autos.ScoreLeaveAndBalance(m_robotDrive, m_altitude, m_extension, m_intake));
    autoSelection.addOption("(9pt) SIDEKICK: Score High & Leave",
        Autos.scoreHighAndLeave(m_robotDrive, m_altitude, m_extension, m_intake));
    autoSelection.addOption("(18pt) RELIABLE: Score High, Balance",
        Autos.scoreHighAndBalance(m_robotDrive, m_altitude, m_extension, m_intake));

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
    // final Trigger startButton = m_operatorController.start();
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

    // startButton.onTrue(new ResetPositionToStart(m_altitude, m_extension));

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
    final Trigger prepareHoldCargo = m_operatorController.a();

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
    return autoSelection.getSelected().andThen(() -> lockWheels());
  }

  public void lockWheels() {
    new RunCommand(m_robotDrive::lock);
  }

  public Command stopIntake() {
    return new InstantCommand(m_intake::stopIntake);
  }

  // public Command getAutonomousCommandForPose() {
  // // Create config for trajectory
  // TrajectoryConfig config = new TrajectoryConfig(
  // AutoConstants.kMaxSpeedMetersPerSecond,
  // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  // // Add kinematics to ensure max speed is actually obeyed
  // .setKinematics(DriveConstants.kDriveKinematics);

  // // An example trajectory to follow. All units in meters.
  // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
  // // Start at the origin facing the +X direction
  // new Pose2d(0, 0, new Rotation2d(0)),
  // // Pass through these two interior waypoints, making an 's' curve path
  // List.of(new Translation2d(.5, .5), new Translation2d(1, -.5)),
  // // End 3 meters straight ahead of where we started, facing forward
  // new Pose2d(1.5, 0, new Rotation2d(0)),
  // config);

  // var thetaController = new ProfiledPIDController(
  // AutoConstants.kPThetaController, 0, 0,
  // AutoConstants.kThetaControllerConstraints);
  // thetaController.enableContinuousInput(-Math.PI, Math.PI);

  // SwerveControllerCommand swerveControllerCommand = new
  // SwerveControllerCommand(
  // exampleTrajectory,
  // m_robotDrive::getPose, // Functional interface to feed supplier
  // DriveConstants.kDriveKinematics,

  // // Position controllers
  // new PIDController(AutoConstants.kPXController, 0, 0),
  // new PIDController(AutoConstants.kPYController, 0, 0),
  // thetaController,
  // m_robotDrive::setModuleStates,
  // m_robotDrive);

  // // Reset odometry to the starting pose of the trajectory.
  // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

  // // Run path following command, then stop at the end.
  // return swerveControllerCommand.andThen(() -> m_robotDrive.stop());
  // }

}
