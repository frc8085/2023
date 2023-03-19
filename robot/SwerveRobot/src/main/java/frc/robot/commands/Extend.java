// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ExtensionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 * Drive the given distance straight (negative values go backwards). Uses a
 * local PID controller to
 * run a simple PID loop that is only enabled while this command is running. The
 * input is the
 * averaged values of the left and right encoders.
 */
public class Extend extends PIDCommand {
    private final Extension m_extension;

    static double kP = 0.05;
    static double kI = 0;
    static double kD = 0;

    /**
     * Create a new TurnToDegreeGyro command.
     *
     * @param distance The distance to drive (inches)
     */
    public Extend(Extension extension, double position) {
        super(new PIDController(kP, kI, kD),
                // Close loop on heading
                extension::getCurrentExtensionPosition,
                // Set reference to target
                position,
                // Pipe output to turn robot
                output -> extension.moveExtension(output));

        // Require the drive
        m_extension = extension;
        addRequirements(m_extension);

        getController().setTolerance(ExtensionConstants.kExtensionPositionTolerance);
    }

    @Override
    public void execute() {
        super.execute();
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        super.initialize();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
