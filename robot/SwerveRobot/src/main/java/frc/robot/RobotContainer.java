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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.OpenIntake;
import frc.robot.commands.PrepareMidDropOff;
import frc.robot.commands.PrepareHighDropOff;
import frc.robot.commands.PrepareTravel;
import frc.robot.commands.PrepareIntake;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;
import frc.robot.subsystems.IntakeCover;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Altitude;
import edu.wpi.first.wpilibj.DriverStation;
// Dashboard
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

        // The robot's subsystems

        private final IntakeCover m_intakeCover = new IntakeCover();
        private final Intake m_intake = new Intake();
        private final Altitude m_Altitude = new Altitude();
        private final Extension m_Extension = new Extension();

        private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_Altitude, m_Extension);

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

                // Configure default commands
                m_robotDrive.setDefaultCommand(
                                // Right Bumper sets a fixed mid speed limit
                                // Right Trigger controls speed
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                m_driverController.getRightBumper(),
                                                                m_driverController.getRightTriggerAxis(),
                                                                // MathUtil.applyDeadband(-m_driverController.getLeftY(),
                                                                // 0.06),
                                                                // MathUtil.applyDeadband(-m_driverController.getLeftX(),
                                                                // 0.06),
                                                                // MathUtil.applyDeadband(-m_driverController.getRightX(),
                                                                // 0.06),

                                                                // new stuff from Rev for SlewRate Limiter
                                                                Math.max(0.0,
                                                                                (Math.abs(m_driverController.getLeftY())
                                                                                                - OIConstants.kDriveDeadband)
                                                                                                / (1.0 - OIConstants.kDriveDeadband))
                                                                                * Math.signum(-m_driverController
                                                                                                .getLeftY()),
                                                                Math.max(0.0,
                                                                                (Math.abs(m_driverController.getLeftX())
                                                                                                - OIConstants.kDriveDeadband)
                                                                                                / (1.0 - OIConstants.kDriveDeadband))
                                                                                * Math.signum(-m_driverController
                                                                                                .getLeftX()),
                                                                Math.max(0.0,
                                                                                (Math.abs(m_driverController
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
                                                                                && m_robotDrive.isWithinSafeLimits()

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

                /** MANUAL OPERATION */
                final Trigger coneIntakeButton = m_operatorController.rightTrigger();
                final Trigger ejectButton = m_operatorController.leftTrigger();

                final Trigger ExtendButton = m_operatorController.povLeft();
                final Trigger RetractButton = m_operatorController.povRight();

                final Trigger ExtendAndLowerButton = m_operatorController.povDownLeft();
                final Trigger RetractAndRaiseButton = m_operatorController.povUpRight();

                final Trigger RaiseButton = m_operatorController.povUp();
                final Trigger LowerButton = m_operatorController.povDown();

                coneIntakeButton.whileTrue(new InstantCommand(m_intake::intakeCone, m_intake))
                                .onFalse(new InstantCommand(m_intake::stopIntake));

                ejectButton.whileTrue(new InstantCommand(m_intake::eject, m_intake))
                                .onFalse(new InstantCommand(m_intake::stopIntake));

                ExtendAndLowerButton
                                .whileTrue(new ParallelCommandGroup(
                                                new InstantCommand(m_Extension::extendExtension, m_Extension),
                                                new InstantCommand(m_Altitude::lowerAltitude, m_Altitude)))
                                .onFalse(new ParallelCommandGroup(
                                                new InstantCommand(m_Extension::stopExtension, m_Extension),
                                                new InstantCommand(m_Altitude::stopAltitude, m_Altitude)));

                RetractAndRaiseButton
                                .whileTrue(new ParallelCommandGroup(
                                                new InstantCommand(m_Extension::retractExtension, m_Extension),
                                                new InstantCommand(m_Altitude::raiseAltitude, m_Altitude)))
                                .onFalse(new ParallelCommandGroup(
                                                new InstantCommand(m_Extension::stopExtension, m_Extension),
                                                new InstantCommand(m_Altitude::stopAltitude, m_Altitude)));

                ExtendButton.whileTrue(new InstantCommand(m_Extension::extendExtension, m_Extension))
                                // .onFalse(new KeepExtensionPosition(m_Extension.getCurrentExtensionPosition(),
                                // m_Extension));
                                .onFalse(new InstantCommand(
                                                () -> m_Extension.keepPosition(
                                                                m_Extension.getCurrentExtensionPosition())));

                RetractButton.whileTrue(new InstantCommand(m_Extension::retractExtension, m_Extension))
                                // .onFalse(new KeepExtensionPosition(m_Extension.getCurrentExtensionPosition(),
                                // m_Extension));
                                .onFalse(new InstantCommand(
                                                () -> m_Extension.keepPosition(
                                                                m_Extension.getCurrentExtensionPosition())));

                RaiseButton
                                .whileTrue(new InstantCommand(m_Altitude::raiseAltitude, m_Altitude))
                                .onFalse(new InstantCommand(
                                                () -> m_Altitude.keepPosition(m_Altitude.getCurrentAltitude())));

                LowerButton
                                .whileTrue(new InstantCommand(m_Altitude::lowerAltitude, m_Altitude))
                                .onFalse(new InstantCommand(
                                                () -> m_Altitude.keepPosition(m_Altitude.getCurrentAltitude())));

                // PRESET POSITIONS
                final Trigger prepareHighDropOffButton = m_operatorController.b();
                final Trigger prepareMidDropOffButton = m_operatorController.x();
                final Trigger prepareTravelButton = m_operatorController.y();
                final Trigger prepareIntakeButton = m_operatorController.a();

                prepareMidDropOffButton.onTrue(new PrepareMidDropOff(m_Extension, m_Altitude));
                prepareHighDropOffButton.onTrue(new PrepareHighDropOff(m_Extension, m_Altitude));
                prepareTravelButton.onTrue(new PrepareTravel(m_Extension, m_Altitude));
                prepareIntakeButton.onTrue(new PrepareIntake(m_Extension, m_Altitude));

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
                                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(3, 0, new Rotation2d(0)),
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
                return swerveControllerCommand.andThen(() -> m_robotDrive.drive(false, 0, 0, 0, 0, false, false));
        }
}
