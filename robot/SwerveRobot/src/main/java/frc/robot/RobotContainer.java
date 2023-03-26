// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AltitudeConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ExtensionConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos.Sidekick.AutoSidekick;
import frc.robot.commands.Autos.SuperHero.AutoSuperHero;
import frc.robot.commands.AutoTurnToDegreeGyro;
import frc.robot.commands.DriverShootCube;
import frc.robot.commands.DropCube;
import frc.robot.commands.IntakeCargo;
import frc.robot.commands.IntakeCargoFromDoubleSubstation;
import frc.robot.commands.IntakeCargoFromSingleSubstation;
import frc.robot.commands.MoveToHighConeDropOff;
import frc.robot.commands.MoveToMidConeDropOff;
import frc.robot.commands.MoveToTravel;
import frc.robot.commands.MoveToTravelAfterIntake;
import frc.robot.commands.MoveToTravelAfterScoring;
import frc.robot.commands.PrepareIntake;
import frc.robot.commands.ScoreBasedOnPosition;
import frc.robot.commands.ScoreHighCube;
import frc.robot.commands.ScoreMidCube;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  protected SendableChooser<Command> autoSelection = new SendableChooser<>();

  // The robot's subsystems
  private final Extension m_extension = new Extension();
  private final Altitude m_altitude = new Altitude(m_extension);
  private final Intake m_intake = new Intake();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_altitude, m_extension);

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Silence Joystick Warnings
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure the Auto Selector
    configureAuto();
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // Right Trigger Controls Speed
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                m_driverController.getRightTriggerAxis(),
                MathUtil.applyDeadband(m_driverController.getLeftY(),
                    OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(m_driverController.getLeftX(),
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(),
                    OIConstants.kDriveDeadband),
                true, m_robotDrive
                    .isWithinSafeDrivingLimits()),
            m_robotDrive));
  }

  private void configureAuto() {

    autoSelection.setDefaultOption("(15pt) SIDEKICK: Score High, Leave, Pickup Cube, Score High ",
        new AutoSidekick(m_robotDrive, m_altitude, m_extension, m_intake));
    autoSelection.addOption("(19pt) SUPERHERO: Score High, Leave, Pickup Cube, Dock",
        new AutoSuperHero(m_robotDrive, m_altitude, m_extension, m_intake));

    // Put the chooser on the dashboard
    SmartDashboard.putData("Auto Routine", autoSelection);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    /** Drive Lock **/
    final Trigger lockWheels = m_driverController.leftBumper();

    lockWheels.toggleOnTrue(new RunCommand(
        () -> m_robotDrive.lock(),
        m_robotDrive));

    // Cube Low Shot on driver control
    final Trigger lowCubeEjectButton = m_driverController.leftTrigger();
    final Trigger cubeShootButton = m_driverController.rightBumper();
    // lowCubeEjectButton.onTrue(new InstantCommand(m_intake::stopIntake));
    lowCubeEjectButton.onTrue(new DropCube(m_altitude, m_extension, m_intake));

    cubeShootButton.onTrue(new DriverShootCube(m_altitude, m_extension,
        m_intake));

    final Trigger zeroHeadingButton = m_driverController.start();
    zeroHeadingButton.onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // Commands to face the robot in different drections

    final Trigger AutoFaceIntakeRight = m_driverController.b();
    final Trigger AutoFaceIntakeUp = m_driverController.y();
    final Trigger AutoFaceIntakeLeft = m_driverController.x();
    final Trigger AutoFaceIntakeDown = m_driverController.a();

    AutoFaceIntakeRight.onTrue(new AutoTurnToDegreeGyro(
        90, m_robotDrive, false));

    AutoFaceIntakeLeft.onTrue(
        new AutoTurnToDegreeGyro(
            -90, m_robotDrive, false));

    AutoFaceIntakeDown.onTrue(
        new AutoTurnToDegreeGyro(
            0, m_robotDrive, false));

    AutoFaceIntakeUp.onTrue(
        new AutoTurnToDegreeGyro(
            180, m_robotDrive, false));

    /** OPERATOR COMMANDS **/
    final Trigger startButton = m_operatorController.start();
    final Trigger intakeButton = m_operatorController.rightTrigger();
    final Trigger manualIntakeButton = m_operatorController.rightBumper();
    final Trigger ejectButton = m_operatorController.leftTrigger();
    final Trigger highCubeEjectButton = m_operatorController.leftBumper();
    // final Trigger midCubeEjectButton = m_operatorController.;
    final Trigger setDoubleSubstationButton = m_operatorController.povUp();
    final Trigger setSingleSubstationButton = m_operatorController.povDown();

    final Trigger testExtension = m_operatorController.povRight();

    final Trigger ExtendButton = m_operatorController.axisLessThan(5, -.25);
    final Trigger RetractButton = m_operatorController.axisGreaterThan(5, .25);
    final Trigger RaiseButton = m_operatorController.axisLessThan(1, -.25);
    final Trigger LowerButton = m_operatorController.axisGreaterThan(1, .25);

    final Trigger testAuto = m_operatorController.povLeft();

    testAuto.onTrue(new AutoSidekick(m_robotDrive, m_altitude, m_extension, m_intake));

    testExtension.onTrue(new InstantCommand(
        () -> m_extension.keepPositionInches(
            ExtensionConstants.kExtensionPositionInchesIntakeOut)));

    startButton.onTrue(Commands.sequence(
        new InstantCommand(() -> m_altitude.reset()),
        new InstantCommand(() -> m_extension.reset())));

    // Intake button will run the intake then hold game piece when released
    intakeButton.whileTrue(new IntakeCargo(m_altitude, m_extension, m_intake))
        .onFalse(new ParallelCommandGroup(
            new InstantCommand(() -> m_intake.holdCargo()),
            new MoveToTravelAfterIntake(m_extension, m_altitude)));

    manualIntakeButton.whileTrue(
        new InstantCommand(() -> m_intake.intakeCube()))
        .onFalse(new InstantCommand(() -> m_intake.holdCargo()));

    setDoubleSubstationButton
        .whileTrue(new IntakeCargoFromDoubleSubstation(m_altitude, m_extension,
            m_intake))
        .onFalse(new ParallelCommandGroup(
            new InstantCommand(() -> m_intake.holdCargo()),
            new MoveToTravelAfterScoring(m_extension, m_altitude)));

    setSingleSubstationButton
        .whileTrue(new IntakeCargoFromSingleSubstation(m_altitude, m_extension,
            m_intake))
        .onFalse(new ParallelCommandGroup(
            new InstantCommand(() -> m_intake.holdCargo()),
            new MoveToTravelAfterScoring(m_extension, m_altitude)));

    ejectButton.onTrue(new ScoreBasedOnPosition(m_altitude, m_extension,
        m_intake));

    highCubeEjectButton.onTrue(new ScoreHighCube(m_altitude, m_extension,
        m_intake));

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
    final Trigger prepareHoldCargo = m_operatorController.a();

    prepareMidDropOffButton.onTrue(new MoveToMidConeDropOff(m_extension,
        m_altitude))
        .onFalse(new SequentialCommandGroup(
            new WaitUntilCommand(
                () -> m_extension.ExtensionIsInMidScoringPosition()),
            new InstantCommand(
                () -> m_altitude.keepPosition(
                    AltitudeConstants.kAltitudeMidDropOffPosition))));

    prepareHighDropOffButton.onTrue(new MoveToHighConeDropOff(m_extension,
        m_altitude))
        .onFalse(new SequentialCommandGroup(
            new WaitUntilCommand(
                () -> m_extension.ExtensionIsInHighScoringPosition()),
            new InstantCommand(
                () -> m_altitude.keepPosition(
                    AltitudeConstants.kAltitudeHighDropOffPosition))));
    prepareTravelButton.onTrue(new MoveToTravel(m_extension, m_altitude));

  }

  // TODO: Make sure robot follows the path properly
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommandOriginal() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // Travel backwards through our trajectory
    config.setReversed(true);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, Rotation2d.fromDegrees(-180)),
        // Pass through these two interior waypoints, making an 's' curve path
        // NOTE: MUST have a waypoint. CANNOT be a straight line.
        List.of(new Translation2d(1, 0.01)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, -0.1, Rotation2d.fromDegrees(1)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.zeroHeading();
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.stop());
  }

  public Command getAutonomousCommand() {
    m_robotDrive.zeroHeading();
    m_robotDrive.resetOdometry(new Pose2d());
    return new AutoSidekick(m_robotDrive, m_altitude, m_extension, m_intake);
    // return autoSelection.getSelected();
  }

}