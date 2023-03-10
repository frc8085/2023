// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 * Drive the given distance straight (negative values go backwards). Uses a
 * local PID controller to
 * run a simple PID loop that is only enabled while this command is running. The
 * input is the
 * averaged values of the left and right encoders.
 */
public class AutoTurnToDegreeGyro extends PIDCommand {
    private final DriveSubsystem m_drive;

    static double kP = 0.01;
    static double kI = 0;
    static double kD = 0.001;

    /**
     * Create a new TurnToDegreeGyro command.
     *
     * @param distance The distance to drive (inches)
     */
    public AutoTurnToDegreeGyro(double degree, DriveSubsystem drive) {
        super(new PIDController(kP, kI, kD),
                // Close loop on heading
                drive::getHeading,
                // Set reference to target
                degree,
                // Pipe output to turn robot
                output -> drive.turn(output));

        // Require the drive
        m_drive = drive;
        addRequirements(m_drive);

        getController().setTolerance(AutoConstants.kAutoGyroTolerance);
    }

    @Override
    public void execute() {
        super.execute();
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        // Get everything in a safe starting state.
        m_drive.resetOdometry(new Pose2d());
        m_drive.zeroHeading();
        super.initialize();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
