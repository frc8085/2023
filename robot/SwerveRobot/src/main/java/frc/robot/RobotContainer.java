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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.OpenIntake;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import frc.robot.subsystems.IntakeCover;
import frc.robot.utilities.JoystickAxisButton;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;
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
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    private final IntakeCover m_intakeCover = new IntakeCover();
    private final Intake m_intake = new Intake();
    private final Elevator m_Elevator = new Elevator();

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

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
                                // MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.06),
                                // MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.06),
                                // MathUtil.applyDeadband(-m_driverController.getRightX(), 0.06),

                                // new stuff from Rev for SlewRate Limiter
                                Math.max(0.0,
                                        (Math.abs(m_driverController.getLeftY()) - OIConstants.kDriveDeadband)
                                                / (1.0 - OIConstants.kDriveDeadband))
                                        * Math.signum(-m_driverController.getLeftY()),
                                Math.max(0.0,
                                        (Math.abs(m_driverController.getLeftX()) - OIConstants.kDriveDeadband)
                                                / (1.0 - OIConstants.kDriveDeadband))
                                        * Math.signum(-m_driverController.getLeftX()),
                                Math.max(0.0,
                                        (Math.abs(m_driverController.getRightX()) - OIConstants.kDriveDeadband)
                                                / (1.0 - OIConstants.kDriveDeadband))
                                        * Math.signum(-m_driverController.getRightX()),
                                true,
                                true),
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

        // Create fake button to correspond to right trigger pressed
        final JoystickAxisButton coneIntakeButton = new JoystickAxisButton("Shoot",
                m_operatorController::getRightTriggerAxis, 0.5);

        // Create fake button to correspond to left trigger pressed
        final JoystickAxisButton ejectButton = new JoystickAxisButton("Shoot", m_operatorController::getLeftTriggerAxis,
                0.5);

        // final JoystickButton coneIntakeButton = new
        // JoystickButton(m_operatorController, Button.kB.value);
        // final JoystickButton ejectButton = new JoystickButton(m_operatorController,
        // Button.kX.value);

        coneIntakeButton.whileTrue(
                new InstantCommand(m_intake::intakeCone, m_intake));
        coneIntakeButton.onFalse(new InstantCommand(m_intake::stopIntake));
        ejectButton.whileTrue(
                new InstantCommand(m_intake::eject, m_intake));
        ejectButton.onFalse(new InstantCommand(m_intake::stopIntake));

        /** MANUAL OPERATION */
        final JoystickButton armExtendButton = new JoystickButton(m_operatorController, Button.kRightBumper.value);
        final JoystickButton armRetractButton = new JoystickButton(m_operatorController, Button.kLeftBumper.value);

        final JoystickButton elevatorRaiseButton = new JoystickButton(m_operatorController, Button.kY.value);
        final JoystickButton elevatorLowerButton = new JoystickButton(m_operatorController, Button.kA.value);

        armExtendButton.whileTrue(
                new InstantCommand(m_Elevator::extendElevatorArm, m_Elevator));
        armExtendButton.onFalse(
                new InstantCommand(m_Elevator::stopArm, m_Elevator));

        armRetractButton.whileTrue(
                new InstantCommand(m_Elevator::retractElevatorArm, m_Elevator));
        armRetractButton.onFalse(
                new InstantCommand(m_Elevator::stop, m_Elevator));

        elevatorRaiseButton.whileTrue(
                new InstantCommand(m_Elevator::raiseElevator, m_Elevator));

        elevatorRaiseButton.onFalse(
                new SequentialCommandGroup(
                        new InstantCommand(m_Elevator::stopElevator, m_Elevator)
                // new MaintainAltitude(() -> m_elevator.getCurrentAltitude(), m_Elevator)

                ));

        elevatorLowerButton.whileTrue(
                new InstantCommand(m_Elevator::lowerElevator, m_Elevator));
        elevatorLowerButton.onFalse(
                new InstantCommand(m_Elevator::stopElevator, m_Elevator));

        new JoystickButton(m_driverController, Button.kLeftBumper.value)
                .whileTrue(new RunCommand(
                        () -> m_robotDrive.lock(),
                        m_robotDrive));
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
