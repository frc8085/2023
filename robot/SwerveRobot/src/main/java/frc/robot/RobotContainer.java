// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.Constants.AltitudeConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.OpenIntake;
import frc.robot.commands.PrepareMidDropOff;
import frc.robot.commands.PrepareHighConeDropOff;
import frc.robot.commands.PrepareTravel;
import frc.robot.commands.PrepareTravelAfterIntake;
import frc.robot.commands.PrepareTravelAfterScoring;
import frc.robot.commands.ScoreBasedOnAltitude;
import frc.robot.commands.ScoreMidCube;
import frc.robot.commands.ScoreHighCube;
import frc.robot.commands.PrepareIntake;
import frc.robot.commands.RunIntakeCargo;
import frc.robot.commands.RunIntakeCargoFromDoubleSubstation;
import frc.robot.commands.RunIntakeCargoFromSingleSubstation;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;
import frc.robot.subsystems.IntakeCover;
import frc.robot.subsystems.Altitude;
import edu.wpi.first.wpilibj.DriverStation;
// Dashboard
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        private double timeWhenIntakeReleased;
        // The robot's subsystems
        private final IntakeCover m_intakeCover = new IntakeCover();
        // private final IntakeNoPID m_intake = new IntakeNoPID();
        private final Intake m_intake = new Intake();
        private final Extension m_extension = new Extension();
        private final Altitude m_altitude = new Altitude(m_extension);
        private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_altitude, m_extension);

        // The driver's controller
        XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

        // The operator's controller
        CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                DriverStation.silenceJoystickConnectionWarning(true);
                System.out.println("FMS? " + DriverStation.isFMSAttached());

                // Configure the button bindings
                configureButtonBindings();

                m_intakeCover.setDefaultCommand(new OpenIntake(m_intakeCover));

                // Reset heading before we start
                m_robotDrive.zeroHeading();
                m_robotDrive.calibrate();

                m_extension.moveToStartingPosition();
                m_altitude.moveToStartingPosition();

                // Configure default commands
                m_robotDrive.setDefaultCommand(
                                // Right Bumper sets a reduced max speed limit
                                // Right Trigger controls speed
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(() -> m_robotDrive.drive(
                                                m_driverController.getRightBumper(),
                                                m_driverController.getRightTriggerAxis(),
                                                // MathUtil.applyDeadband(-m_driverController.getLeftY(),
                                                // 0.06),
                                                // MathUtil.applyDeadband(-m_driverController.getLeftX(),
                                                // 0.06),
                                                // MathUtil.applyDeadband(-m_driverController.getRightX(),
                                                // 0.06),

                                                // new stuff from Rev for SlewRate Limiter
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
                                                                * Math.signum(m_driverController
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
                /** DRIVE LOCK **/
                new JoystickButton(m_driverController, Button.kLeftBumper.value)
                                .whileTrue(new RunCommand(() -> m_robotDrive.lock(), m_robotDrive));

                /** OPERATOR COMMANDS **/
                final Trigger intakeButton = m_operatorController.rightTrigger();
                final Trigger manualIntakeButton = m_operatorController.rightBumper();
                final Trigger ejectButton = m_operatorController.leftTrigger();
                final Trigger midCubeEjectButton = m_operatorController.leftBumper();
                final Trigger setDoubleSubstationButton = m_operatorController.povUp();
                final Trigger setSingleSubstationButton = m_operatorController.povDown();

                /** MANUAL OPERATION **/
                final Trigger ExtendButton = m_operatorController.axisLessThan(5, -.25);
                final Trigger RetractButton = m_operatorController.axisGreaterThan(5, .25);

                final Trigger RaiseButton = m_operatorController.axisLessThan(1, -.25);
                final Trigger LowerButton = m_operatorController.axisGreaterThan(1, .25);

                // trying PID
                // intakeButton.whileTrue(new InstantCommand(() -> m_intake.intakeCone()))
                // .onFalse(new InstantCommand(() -> m_intake.holdCargo()));

                intakeButton.whileTrue(
                                new ConditionalCommand(
                                                new RunIntakeCargo(m_altitude, m_extension, m_intake),
                                                new InstantCommand(),
                                                () -> timeSince(timeWhenIntakeReleased) > IntakeConstants.kIntakeSafetyPressWaitTime))
                                .onFalse(new ParallelCommandGroup(
                                                new InstantCommand(() -> timeWhenIntakeReleased = Timer.getMatchTime()),
                                                new InstantCommand(() -> m_intake.holdCargo()),
                                                new PrepareTravelAfterIntake(m_extension, m_altitude)));

                manualIntakeButton.whileTrue(new InstantCommand(() -> m_intake.intakeCube()))
                                .onFalse(new InstantCommand(() -> m_intake.holdCargo()));

                setDoubleSubstationButton
                                .whileTrue(new RunIntakeCargoFromDoubleSubstation(m_altitude, m_extension, m_intake))
                                .onFalse(new ParallelCommandGroup(
                                                new InstantCommand(() -> m_intake.holdCargo()),
                                                new PrepareTravelAfterScoring(m_extension, m_altitude)));

                setSingleSubstationButton
                                .whileTrue(new RunIntakeCargoFromSingleSubstation(m_altitude, m_extension, m_intake))
                                .onFalse(new ParallelCommandGroup(
                                                new InstantCommand(() -> m_intake.holdCargo()),
                                                new PrepareTravelAfterScoring(m_extension, m_altitude)));

                ejectButton.onTrue(new ScoreBasedOnAltitude(m_altitude, m_extension,
                                m_intake));

                midCubeEjectButton.onTrue(new ScoreMidCube(m_altitude, m_extension, m_intake));

                ExtendButton.whileTrue(new InstantCommand(m_extension::extendExtension, m_extension))
                                // .onFalse(new KeepExtensionPosition(m_extension.getCurrentExtensionPosition(),
                                // m_extension));
                                .onFalse(new InstantCommand(
                                                () -> m_extension.keepPosition(
                                                                m_extension.getCurrentExtensionPosition())));

                RetractButton.whileTrue(new InstantCommand(m_extension::retractExtension, m_extension))
                                // .onFalse(new KeepExtensionPosition(m_extension.getCurrentExtensionPosition(),
                                // m_extension));
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
                final Trigger prepareIntakeButton = m_operatorController.a();
                // final Trigger prepareShelfPickupButton = m_operatorController.start();

                prepareMidDropOffButton.onTrue(new PrepareMidDropOff(m_extension, m_altitude));
                prepareHighDropOffButton.onTrue(new PrepareHighConeDropOff(m_extension, m_altitude))
                                .onFalse(new SequentialCommandGroup(
                                                new WaitUntilCommand(
                                                                () -> m_extension.ExtensionIsInHighScoringPosition()),
                                                new InstantCommand(
                                                                () -> m_altitude.keepPosition(
                                                                                AltitudeConstants.kAltitudeHighDropOffPosition))));
                prepareTravelButton.onTrue(new PrepareTravel(m_extension, m_altitude));
                prepareIntakeButton.onTrue(new PrepareIntake(m_extension, m_altitude));

        }

        public double timeSince(double startTime) {
                return startTime - Timer.getMatchTime();
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // Create config for trajectory
                TrajectoryConfig config = new TrajectoryConfig(
                                AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                // Add kinematics to ensure max speed is actually obeyed
                                .setKinematics(DriveConstants.kDriveKinematics);

                // An example trajectory to follow. All units in meters.
                Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(2.8, 0)),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(1.15, 0, new Rotation2d(0)),
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
                m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

                // Run path following command, then stop at the end.
                return swerveControllerCommand.andThen(
                                () -> m_robotDrive.lock());
                // drive(false, 0, 0, 0, 0, false, false));
        }
}
