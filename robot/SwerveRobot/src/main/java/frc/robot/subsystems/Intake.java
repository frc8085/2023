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

import static frc.robot.Constants.IntakeConstants;

import java.util.HashMap;
import java.util.Map;

public class Intake extends SubsystemBase {
  private boolean TUNING_MODE = false;
  private final CANSparkMax m_intakeMotor;
  private SparkMaxPIDController m_intakePIDController;
  private RelativeEncoder m_intakeEncoder;
  private double kIntakeSetPoint, kPIntake, kIIntake, kDIntake, kIntakeMaxOutput, kIntakeMinOutput;

  /**
   * CargoModes Mode definition
   * currentCargoMode is an integer corresponding to the current type of
   * cargo
   * CargoModes translates the integer into a string so we can display it
   * in the dashboard
   */

  private int currentCargoMode = IntakeConstants.kIntakeOffSpeed;

  private static final Map<Integer, String> CargoModes = new HashMap<Integer, String>() {
    {
      put(IntakeConstants.kCargoNone, "No cargo type selected");

      put(IntakeConstants.kCargoCubeIntake, "Intake Cube");
      put(IntakeConstants.kCargoCubeEject, "Eject Cube");

      put(IntakeConstants.kCargoConeIntake, "Intake Cone");
      put(IntakeConstants.kCargoConeEject, "Eject Cone");
    }
  };

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

  /** The shooter subsystem for the robot. */
  public Intake() {
    m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakePort, MotorType.kBrushless);
    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setIdleMode(IdleMode.kCoast);
    m_intakeMotor.setSmartCurrentLimit(SubsystemMotorConstants.kMotorCurrentLimit550);
    m_intakeEncoder = m_intakeMotor.getEncoder();
    m_intakePIDController = m_intakeMotor.getPIDController();

    // PID coefficients
    kPIntake = 0.0004;
    kIIntake = 0;
    kDIntake = 0.004;
    kIntakeMaxOutput = 0;
    kIntakeMinOutput = -1;
    kIntakeSetPoint = IntakeConstants.kIntakeTargetRPM[0];

    // Set PID coefficients
    m_intakePIDController.setP(kPIntake);
    m_intakePIDController.setI(kIIntake);
    m_intakePIDController.setD(kDIntake);
    m_intakePIDController.setOutputRange(kIntakeMinOutput, kIntakeMaxOutput);

    // Add relevant displays to the Operator dashboard
    configureOperatorDashboard();

    // If we're fine-tuning PID Constants, the display them on the dashboard
    if (TUNING_MODE) {
      addPIDToDashboard();
    }

  }

  private void configureOperatorDashboard() {
    // Add the selected shooting mode to the Operator dashboard
    SmartDashboard.putString("Cargo Mode", CargoModes.get(currentCargoMode));

    // Add the setpoint but only if in TUNINGMODE
    if (TUNING_MODE) {
      SmartDashboard.putNumber("Intake Setpoint", kIntakeSetPoint);
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

  @Override
  public void periodic() {
    // If we're fine-tuning PID Constants, read and apply updates from the dashboard
    if (TUNING_MODE) {
      readPIDTuningFromDashboard();
      SmartDashboard.putNumber("Intake Encoder velocity", m_intakeEncoder.getVelocity());

    }

    // add intake encoder position into log
    logIntakeData();

  }

  public void setSetpoint(double setPoint) {
    m_intakePIDController.setReference(kIntakeSetPoint, ControlType.kVelocity);
  }

  public boolean atSetpoint() {
    double encoderValue = m_intakeEncoder.getVelocity();
    // double tolerance = Math.abs(kShooterToleranceRPMPercent * kSetPoint);
    double tolerance = 300;
    double setpoint = kIntakeSetPoint;
    double minLimit = setpoint - tolerance;
    double maxLimit = setpoint + tolerance;

    boolean withinLimits =
        // Don't consider us at setpoint for the 'motor off' case
        setpoint != 0 &&
        // Otherwise check if we're within limits
            encoderValue >= minLimit
            && encoderValue <= maxLimit;

    return withinLimits;
  }

  public void holdCargo() {
    m_intakePIDController.setReference(m_intakeEncoder.getPosition(), CANSparkMax.ControlType.kPosition);
  }

  public void setCargoMode(int mode) {
    currentCargoMode = mode;
    updateSetpoint();
  }

  private void updateSetpoint() {
    // Update the setpoint to whatever the current cargo mode is
    setSetpoint(IntakeConstants.kIntakeTargetRPM[currentCargoMode]);
  }

}
