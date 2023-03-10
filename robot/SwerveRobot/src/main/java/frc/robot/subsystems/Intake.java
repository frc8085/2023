// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemMotorConstants;
import frc.robot.subsystems.LEDs.GamePiece;

import static frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private boolean TUNING_MODE = false;
    private final CANSparkMax m_intakeMotor;
    private RelativeEncoder m_intakeEncoder;

    // Determine current intake encoder position
    public double CurrentIntakeEncoderPosition() {
        return m_intakeEncoder.getPosition();
    }

    // PID
    private SparkMaxPIDController m_intakePIDController;
    static double kPIntake = .5;
    static double kIIntake = 0;
    static double kDIntake = 0.;
    static double kFFIntake = 0;
    static double kIntakeMaxOutput = .25;
    static double kIntakeMinOutput = -.25;

    // Eject Wait Time
    static double kEjectWaitTime = IntakeConstants.kEjectWaitTime;

    /** The intake subsystem for the robot. */
    public Intake() {
        m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakePort, MotorType.kBrushless);
        m_intakeMotor.setOpenLoopRampRate(IntakeConstants.kRampRate);
        m_intakeMotor.restoreFactoryDefaults();
        m_intakeMotor.setIdleMode(IdleMode.kCoast);
        m_intakeMotor.setSmartCurrentLimit(SubsystemMotorConstants.kMotorCurrentLimit550);
        m_intakeEncoder = m_intakeMotor.getEncoder();
        m_intakePIDController = m_intakeMotor.getPIDController();
        m_intakeMotor.burnFlash();

        // Set PID coefficients
        m_intakePIDController.setP(kPIntake);
        m_intakePIDController.setI(kIIntake);
        m_intakePIDController.setD(kDIntake);
        m_intakePIDController.setFF(kFFIntake);
        m_intakePIDController.setOutputRange(kIntakeMinOutput, kIntakeMaxOutput);

        // If we're fine-tuning PID Constants, the display them on the dashboard
        if (TUNING_MODE) {
            addPIDToDashboard();
            addEjectWaitTimeToDashboard();
        }

    }

    private void addPIDToDashboard() {
        // Display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("Intake P Gain", kPIntake);
        SmartDashboard.putNumber("Intake I Gain", kIIntake);
        SmartDashboard.putNumber("Intake D Gain", kDIntake);
        SmartDashboard.putNumber("Intake Max Output", kIntakeMaxOutput);
        SmartDashboard.putNumber("Intake Min Output", kIntakeMinOutput);

    }

    private void addEjectWaitTimeToDashboard() {
        SmartDashboard.putNumber("Eject Wait Time", kEjectWaitTime);
    }

    private void readPIDTuningFromDashboard() {

        // Read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("Intake P Gain", 0);
        double i = SmartDashboard.getNumber("Intake I Gain", 0);
        double d = SmartDashboard.getNumber("Intake D Gain", 0);
        double max = SmartDashboard.getNumber("Intake Max Output", 0);
        double min = SmartDashboard.getNumber("Intake Min Output", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller
        if ((p != kPIntake)) {
            m_intakePIDController.setP(p);
            kPIntake = p;
        }
        if ((i != kIIntake)) {
            m_intakePIDController.setI(i);
            kIIntake = i;
        }
        if ((d != kDIntake)) {
            m_intakePIDController.setD(d);
            kDIntake = d;
        }
        if ((max != kIntakeMaxOutput) || (min != kIntakeMinOutput)) {
            m_intakePIDController.setOutputRange(min, max);
            kIntakeMinOutput = min;
            kIntakeMaxOutput = max;
        }
    }

    public void readEjectWaitTimeFromDashboard() {
        // Read Eject Wait Time from SmartDashboard
        double t = SmartDashboard.getNumber("Eject Wait Time", IntakeConstants.kEjectWaitTime);

        // if Eject Wait Time on SmartDashboard has changed, write new values to the
        // controller
        if ((t != kEjectWaitTime)) {
            kEjectWaitTime = t;
        }
    }

    @Override
    public void periodic() {
        // If we're fine-tuning PID Constants, read and apply updates from the dashboard
        if (TUNING_MODE) {
            readPIDTuningFromDashboard();
            readEjectWaitTimeFromDashboard();
            SmartDashboard.putNumber("Intake Encoder position", m_intakeEncoder.getPosition());

            // add intake encoder position into log
            // logIntakeData();

        }

    }

    // log into console the intake encoder position
    // public void logIntakeData() {
    // System.out.println("Position " + CurrentIntakeEncoderPosition());
    // }

    /** Resets the Intake encoder to currently read a position of 0. */
    public void reset() {
        m_intakeEncoder.setPosition(0);
    }

    public void holdCargo() {
        LEDs.getInstance().selectedGamePiece = GamePiece.NONE;
        m_intakePIDController.setReference(CurrentIntakeEncoderPosition(), ControlType.kPosition);
    }

    // Open loop stuff
    // Run the intake forward at the CONE speed
    public void intakeCone() {
        LEDs.getInstance().selectedGamePiece = GamePiece.CONE;
        m_intakeMotor.set(IntakeConstants.kIntakeConePower);
    }

    // Run the intake forward at the CUBE speed
    public void intakeCube() {
        LEDs.getInstance().selectedGamePiece = GamePiece.CUBE;
        m_intakeMotor.set(IntakeConstants.kIntakeCubePower);
    }

    // Run the intake reverse to eject CONE
    public void ejectCone() {
        m_intakeMotor.set(-IntakeConstants.kEjectConePower);
    }

    // Run the intake reverse to eject CUBE
    public void ejectCube() {
        m_intakeMotor.set(-IntakeConstants.kEjectCubePower);
    }

    // Stop the intake
    public void stopIntake() {
        m_intakeMotor.set(0);
    }
}
