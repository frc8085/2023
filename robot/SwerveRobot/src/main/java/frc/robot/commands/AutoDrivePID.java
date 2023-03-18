// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Drive until we balance. Uses a local PID controller to
 * run a simple PID loop that is only enabled while this command is running.
 * The input is the current pitch reading from our Gyro
 */
public class AutoDrivePID extends PIDCommand {
    private boolean TUNING_MODE = true;
    private final DriveSubsystem m_drive;
    private static double metersPerSecondTolerance = 0.1;
    private static double metersPerSecond = .05;
    private static double m_meters;

    static double kP = 10;
    static double kI = 0;
    static double kD = 1;

    /**
     * Create a new AutoDrivePID command.
     */
    public AutoDrivePID(DriveSubsystem drive, double meters) {
        super(new PIDController(kP, kI, kD),
                // Close loop on velocity
                drive::getCurrentVelocity,
                // Set reference to target
                metersPerSecond,
                // Pipe output to drive robot
                output -> drive.drive(
                        false,
                        // The PID output to control our speed
                        output,
                        // If reading positive pitch, drive backwards.
                        // If reading negative pitch, drive forwards
                        Math.signum(meters),
                        0,
                        0,
                        true,
                        false));

        // Require the drive
        m_drive = drive;
        m_meters = meters;
        addRequirements(m_drive);

        // Set desired pitch tolerance in degrees
        // getController().setTolerance(metersPerSecondTolerance);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        super.initialize();
        addPIDToDashboard();
        // Get everything in a safe starting state.
        m_drive.resetOdometry(new Pose2d());
        m_drive.zeroHeading();
    }

    @Override
    public void execute() {
        super.execute();
        if (TUNING_MODE) {
            readPIDTuningFromDashboard();
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        // TODO: Test. If we fail to balance, make sure we still lock our wheels in Auto
        // boolean timeToLock = Timer.getMatchTime() < 1.5;
        boolean atSetpoint = false;
        // Math.abs(m_drive.getPitch()) < 1;

        if (m_meters > 0) {
            atSetpoint = m_drive.getX() >= m_meters;
        } else {
            atSetpoint = m_drive.getX() <= m_meters;
        }

        return atSetpoint;
    }

    private void addPIDToDashboard() {
        // Display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("DrivePID P Gain", kP);
        SmartDashboard.putNumber("DrivePID I Gain", kI);
        SmartDashboard.putNumber("DrivePID D Gain", kD);

    }

    // Allows us to fine tune PID from the dashboard
    private void readPIDTuningFromDashboard() {
        // Read PID Coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("DrivePID P Gain", kP);
        double i = SmartDashboard.getNumber("DrivePID I Gain", kI);
        double d = SmartDashboard.getNumber("DrivePID D Gain", kD);

        // If PID coefficients on SmartDashboard have changed, write new values to
        // controller
        if ((p != kP)) {
            getController().setP(p);
            kP = p;
        }
        if ((i != kI)) {
            getController().setI(i);
            kI = i;
        }
        if ((d != kD)) {
            getController().setD(d);
            kD = d;
        }
    }

}