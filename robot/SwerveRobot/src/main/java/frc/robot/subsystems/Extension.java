// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtensionConstants;
import frc.robot.Constants.SubsystemMotorConstants;

public class Extension extends SubsystemBase {
  private boolean TUNING_MODE = true;

  /** Creates a new Extension. */

  // Extension motors
  private final CANSparkMax m_extensionMotor = new CANSparkMax(ExtensionConstants.kExtensionMotorPort,
      MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder m_extensionEncoder = m_extensionMotor.getEncoder();

  // Limit Switches
  private SparkMaxLimitSwitch m_extensionLimit;
  private SparkMaxLimitSwitch m_retractionLimit;

  // PID
  private SparkMaxPIDController m_extensionPIDController = m_extensionMotor.getPIDController();

  // Extend PID coefficients
  static int kExtendPIDSlot = 0;
  static double kPExtend = .05;
  static double kIExtend = 0;
  static double kDExtend = 0;
  static double kFFExtend = 0;
  static double kMinOutputExtend = -0.85;
  static double kMaxOutputExtend = 0.85;

  // Retract PID coefficients
  static int kRetractPIDSlot = 1;
  static double kPRetract = .05;
  static double kIRetract = 0;
  static double kDRetract = 0;
  static double kFFRetract = 0;
  static double kMinOutputRetract = -0.85;
  static double kMaxOutputRetract = 0.85;

  public boolean ExtensionRetractionLimitHit() {
    return isRetractionLimitHit();
  }

  public Extension() {

    m_extensionMotor.setIdleMode(IdleMode.kBrake);
    m_extensionMotor.setSmartCurrentLimit((SubsystemMotorConstants.kMotorCurrentLimit));
    m_extensionMotor.setOpenLoopRampRate(ExtensionConstants.kExtensionRampRate);

    m_extensionPIDController.setFeedbackDevice(m_extensionEncoder);

    // ** EXTEND PID: Use Extend coefficients and slotID for Extend PID */
    m_extensionPIDController.setP(kPExtend, kExtendPIDSlot);
    m_extensionPIDController.setI(kIExtend, kExtendPIDSlot);
    m_extensionPIDController.setD(kDExtend, kExtendPIDSlot);
    m_extensionPIDController.setFF(kFFExtend, kExtendPIDSlot);
    m_extensionPIDController.setOutputRange(kMinOutputExtend, kMaxOutputExtend, kExtendPIDSlot);

    // ** RETRACT PID: Use Extend coefficients and slotID for Extend PID */
    m_extensionPIDController.setP(kPRetract, kRetractPIDSlot);
    m_extensionPIDController.setI(kIRetract, kRetractPIDSlot);
    m_extensionPIDController.setD(kDRetract, kRetractPIDSlot);
    m_extensionPIDController.setFF(kFFRetract, kRetractPIDSlot);
    m_extensionPIDController.setOutputRange(kMinOutputRetract, kMaxOutputRetract, kRetractPIDSlot);

    /**
     * A SparkMaxLimitSwitch object is constructed using the getForwardLimitSwitch()
     * or
     * on which direction you would like to limit
     * 
     * Limit switches can be configured to one of two polarities:
     * com.revrobotics.SparkMaxLimitSwitch.SparkMaxLimitSwitch.Type.kNormallyOpen
     * com.revrobotics.SparkMaxLimitSwitch.SparkMaxLimitSwitch.Type.kNormallyClosed
     */
    m_extensionLimit = m_extensionMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_retractionLimit = m_extensionMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.

    m_extensionMotor.burnFlash();
  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();
    ExtensionRetractionLimitHit();
  }

  /** The log method puts interesting information to the SmartDashboard. */
  public void log() {
    if (TUNING_MODE) {
      // SmartDashboard.putBoolean("Fully Extended", m_extensionLimit.isPressed());
      // SmartDashboard.putBoolean("Fully Retracted", m_retractionLimit.isPressed());
      // SmartDashboard.putBoolean("Extension Travel Position",
      // ExtensionIsInTravelPosition());
      // SmartDashboard.putBoolean("Extension Intake Position",
      // ExtensionIsInIntakePosition());
      SmartDashboard.putNumber("Extension Current position",
          getCurrentExtensionPosition());

      readExtendPIDTuningFromDashboard();
      readRetractPIDTuningFromDashboard();
    }
  }

  private void readExtendPIDTuningFromDashboard() {
    // Read Extend PID Coefficients from SmartDashboard
    double pExtend = SmartDashboard.getNumber("Extend P Gain", 0);
    double iExtend = SmartDashboard.getNumber("Extend I Gain", 0);
    double dExtend = SmartDashboard.getNumber("Extend D Gain", 0);
    double ffExtend = SmartDashboard.getNumber("Extend Feed Forward", 0);
    double maxExtend = SmartDashboard.getNumber("Extend Max Output", 0);
    double minExtend = SmartDashboard.getNumber("Extend Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller. Make sure to use the PID Extend slot
    if ((pExtend != kPExtend)) {
      m_extensionPIDController.setP(pExtend, kExtendPIDSlot);
      kPExtend = pExtend;
    }
    if ((iExtend != kIExtend)) {
      m_extensionPIDController.setI(iExtend, kExtendPIDSlot);
      kIExtend = iExtend;
    }
    if ((dExtend != kDExtend)) {
      m_extensionPIDController.setD(dExtend, kExtendPIDSlot);
      kDExtend = dExtend;
    }

    if ((ffExtend != kFFExtend)) {
      m_extensionPIDController.setFF(ffExtend, kExtendPIDSlot);
      kFFExtend = ffExtend;
    }

    if ((maxExtend != kMaxOutputExtend) || (minExtend != kMinOutputExtend)) {
      m_extensionPIDController.setOutputRange(minExtend, maxExtend, kExtendPIDSlot);
      kMinOutputExtend = minExtend;
      kMaxOutputExtend = maxExtend;
    }
  }

  private void readRetractPIDTuningFromDashboard() {
    // Read PID Coefficients from SmartDashboard
    double pRetract = SmartDashboard.getNumber("Retract P Gain", 0);
    double iRetract = SmartDashboard.getNumber("Retract I Gain", 0);
    double dRetract = SmartDashboard.getNumber("Retract D Gain", 0);
    // double izRetract = SmartDashboard.getNumber("Retract I Zone", 0);
    double ffRetract = SmartDashboard.getNumber("Retract Feed Forward", 0);
    double maxRetract = SmartDashboard.getNumber("Retract Max Output", 0);
    double minRetract = SmartDashboard.getNumber("Retract Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller. Make sure to use the PID Retract slot
    if ((pRetract != kPRetract)) {
      m_extensionPIDController.setP(pRetract, kRetractPIDSlot);
      kPRetract = pRetract;
    }
    if ((iRetract != kIRetract)) {
      m_extensionPIDController.setI(iRetract, kRetractPIDSlot);
      kIRetract = iRetract;
    }
    if ((dRetract != kDRetract)) {
      m_extensionPIDController.setD(dRetract, kRetractPIDSlot);
      kDRetract = dRetract;
    }

    if ((ffRetract != kFFRetract)) {
      m_extensionPIDController.setFF(ffRetract, kRetractPIDSlot);
      kFFRetract = ffRetract;
    }

    if ((maxRetract != kMaxOutputRetract) || (minRetract != kMinOutputRetract)) {
      m_extensionPIDController.setOutputRange(minRetract, maxRetract, kRetractPIDSlot);
      kMinOutputRetract = minRetract;
      kMaxOutputRetract = maxRetract;
    }
  }

  // Reset the Extension Encoder when the Retraction Limit is pressed
  public boolean isRetractionLimitHit() {
    return m_retractionLimit.isPressed();
  }

  public void resetExtensionEncoderAtRetractionLimit() {
    m_extensionEncoder.setPosition(1);
  }

  /** ELEVATOR Extension **/
  // Run the elevator Extension motor forward
  public void extendExtension() {
    m_extensionMotor.set(ExtensionConstants.kExtensionExtendSpeed);
  }

  // Run the elevator Extension motor in reverse
  public void retractExtension() {
    m_extensionMotor.set(-ExtensionConstants.kExtensionRetractSpeed);
  }

  // Stop the elevator Extension
  public void stopExtension() {
    m_extensionMotor.set(0);
  }

  // Returns the current position of the Extension
  public double getCurrentExtensionPosition() {
    return m_extensionEncoder.getPosition();
  }

  /**
   * Move to and stay at a certain extension encoder value
   * Use the Extend PID coefficients if we are going up
   * Otherwise use the Retract PID coefficients
   * 
   * @param position The desired extension
   **/
  public void keepPosition(double position) {
    boolean extending = getCurrentExtensionPosition() <= position;
    // int slotID = extending ? kExtendPIDSlot : kRetractPIDSlot;
    int slotID = kExtendPIDSlot;
    // Set the position using the correct SlotID for desired PID controller
    // (extend vs retract)
    m_extensionPIDController.setReference(position, ControlType.kPosition, slotID);

    if (TUNING_MODE) {
      SmartDashboard.putString("EXTENSION MODE", extending ? "EXTENDING" : "RETRACTING");
      SmartDashboard.putNumber("Desired Extension position", position);
    }

  }

  /**
   * Move to and stay at a certain extension in inches
   * Use the Extend PID coefficients if we are going up
   * Otherwise use the Retract PID coefficients
   * 
   * @param position The desired extension
   **/
  public void keepPositionInches(double positionInches) {
    // set position in inches, convert to encoder value
    double position;
    position = positionInches * ExtensionConstants.kExtensionRevolutionsPerInch + 1;

    boolean extending = getCurrentExtensionPosition() <= position;
    // int slotID = extending ? kExtendPIDSlot : kRetractPIDSlot;
    int slotID = kExtendPIDSlot;

    // Set the position using the correct SlotID for desired PID controller
    // (extend vs retract)
    m_extensionPIDController.setReference(position, ControlType.kPosition, slotID);

    if (TUNING_MODE) {
      SmartDashboard.putString("EXTENSION MODE", extending ? "EXTENDING" : "RETRACTING");
      SmartDashboard.putNumber("Desired Extension position", position);
    }
  }

  // Tell Us if Extension as At Positions
  public boolean ExtensionIsInTravelPosition() {
    return m_extensionEncoder
        .getPosition() > (ExtensionConstants.kExtensionPositionFullyRetracted
            - ExtensionConstants.kExtensionPositionTolerance);
  }

  public boolean ExtensionIsInIntakePosition() {
    return m_extensionEncoder.getPosition() < ExtensionConstants.kExtensionPositionIntakeOut
        + ExtensionConstants.kExtensionPositionTolerance
        && m_extensionEncoder.getPosition() > ExtensionConstants.kExtensionPositionIntakeOut
            - ExtensionConstants.kExtensionPositionTolerance;
  }

  public boolean ExtensionIsInMidCubeShootPosition() {
    return m_extensionEncoder.getPosition() < ExtensionConstants.kExtensionPositionMidCubeShooter
        + ExtensionConstants.kExtensionPositionTolerance
        && m_extensionEncoder.getPosition() > ExtensionConstants.kExtensionPositionMidCubeShooter
            - ExtensionConstants.kExtensionPositionTolerance;
  }

  public boolean ExtensionIsInHighCubeShootPosition() {
    return m_extensionEncoder.getPosition() < ExtensionConstants.kExtensionPositionHighCubeShooter
        + ExtensionConstants.kExtensionPositionTolerance
        && m_extensionEncoder.getPosition() > ExtensionConstants.kExtensionPositionHighCubeShooter
            - ExtensionConstants.kExtensionPositionTolerance;
  }

  public boolean ExtensionIsInDropOffReturnPosition() {
    return m_extensionEncoder.getPosition() < ExtensionConstants.kExtensionPositionHighDropOffReturn
        + ExtensionConstants.kExtensionPositionTolerance;
  }

  public boolean ExtensionIsInMidScoringPosition() {
    return m_extensionEncoder.getPosition() < ExtensionConstants.kExtensionPositionMidDropOff
        + ExtensionConstants.kExtensionPositionTolerance &&
        m_extensionEncoder.getPosition() > ExtensionConstants.kExtensionPositionMidDropOff
            - ExtensionConstants.kExtensionPositionTolerance;
  };

  public boolean ExtensionIsInHighScoringPosition() {
    return m_extensionEncoder.getPosition() < ExtensionConstants.kExtensionPositionHighDropOff
        + ExtensionConstants.kExtensionPositionTolerance &&
        m_extensionEncoder.getPosition() > ExtensionConstants.kExtensionPositionHighDropOff
            - ExtensionConstants.kExtensionPositionTolerance;
  };

  public boolean ExtensionIsInReleasePosition() {
    double ReleaseExtensionPosition = setReleaseExtensionPosition();

    return m_extensionEncoder.getPosition() < ReleaseExtensionPosition
        + ExtensionConstants.kExtensionPositionTolerance
        &&
        m_extensionEncoder.getPosition() > ReleaseExtensionPosition
            - ExtensionConstants.kExtensionPositionTolerance;
  };

  public double setReleaseExtensionPosition() {
    return getCurrentExtensionPosition() - ExtensionConstants.kExtensionConeRetractDistance;
  }
}
