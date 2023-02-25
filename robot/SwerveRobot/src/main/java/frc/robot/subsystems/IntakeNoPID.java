// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemMotorConstants;

import static frc.robot.Constants.IntakeConstants;
import static frc.robot.Constants.IntakeNoPIDConstants;

public class IntakeNoPID extends SubsystemBase {
    /** Creates a new Intake. */

    // Intake motor
    private final CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakePort, MotorType.kBrushless);

    // Intake Encoder
    private final RelativeEncoder m_intakeEncoder = m_intakeMotor.getEncoder();

    // Attempt to write position encoder stall detection
    static double intakeEncoder = 0;
    static double kIntakeEncoderTolerance = 1;

    // Determine current intake encoder position
    public double CurrentIntakeEncoderPosition() {
        return m_intakeEncoder.getPosition();
    }

    // Determine current intake encoder velocity
    public double CurrentIntakeEncoderVelocity() {
        return m_intakeEncoder.getVelocity();
    }

    // log into console the intake encoder position & velocity readings
    public void logIntakeData() {
        System.out.println("Velocity " + CurrentIntakeEncoderVelocity());
        System.out.println("Position " + CurrentIntakeEncoderPosition());
    }
    // show on screen the live values so we can plot

    // The Intake Subsystem for the robot
    public IntakeNoPID() {
        m_intakeMotor.setOpenLoopRampRate(IntakeConstants.kRampRate);
        m_intakeMotor.setIdleMode(IdleMode.kCoast);
        m_intakeMotor.setSmartCurrentLimit(SubsystemMotorConstants.kMotorCurrentLimit550);

        m_intakeMotor.burnFlash();
    }

    // Open loop stuff
    // Run the intake forward at the CONE speed
    public void intakeCone() {
        m_intakeMotor.set(IntakeNoPIDConstants.kIntakeConePower);
    }

    // Run the intake forward at the CUBE speed
    public void intakeCube() {
        m_intakeMotor.set(IntakeNoPIDConstants.kIntakeCubePower);
    }

    // Run the intake reverse to eject CONE
    public void ejectCone() {
        m_intakeMotor.set(-IntakeNoPIDConstants.kEjectConePower);
    }

    // Run the intake reverse to eject CUBE
    public void ejectCube() {
        m_intakeMotor.set(-IntakeNoPIDConstants.kEjectCubePower);
    }

    // Run the intake forward to hold the CUBE speed
    public void holdCube() {
        m_intakeMotor.set(IntakeNoPIDConstants.kIntakeHoldCubePower);
    }

    // Stop the intake
    public void stopIntake() {
        m_intakeMotor.set(0);
    }

    // Log info for smart dashboard
    public void log() {
        SmartDashboard.putNumber("Current Intake Encoder Velocity", CurrentIntakeEncoderVelocity());
        SmartDashboard.putNumber("Current Intake Encoder Position", CurrentIntakeEncoderPosition());
    }

    /** Call log method every loop. */
    @Override
    public void periodic() {
        log();

        // add intake encoder position into log
        logIntakeData();

    }

}
