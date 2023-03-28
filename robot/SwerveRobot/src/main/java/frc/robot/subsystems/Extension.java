// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TuningModeConstants;

import static frc.robot.Constants.ExtensionConstants;
import static frc.robot.Constants.SubsystemMotorConstants;

public class Extension extends SubsystemBase {
  private boolean TUNING_MODE = TuningModeConstants.kExtensionTuning;

  /** Creates a new Extension. */

  // Extension motors
  private final CANSparkMax m_extensionMotor = new CANSparkMax(ExtensionConstants.kExtensionMotorPort,
      MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder m_extensionEncoder = m_extensionMotor.getEncoder();

  // Limit Switches
  private SparkMaxLimitSwitch m_extensionLimit;
  private SparkMaxLimitSwitch m_retractionLimit;

  // Extension Limit Switches are on by default is locked by default
  private boolean extensionLimitSwitchIsDisabled = false;

  /* Ignore Limit Switch */
  public void disableExtensionLimitSwitch() {
    extensionLimitSwitchIsDisabled = true;
  }

  /* Enable Limit Switch */
  public void activateExtensionLimitSwitch() {
    extensionLimitSwitchIsDisabled = false;
  }

  public boolean extensionLimitSwitchIsDisabled() {
    return extensionLimitSwitchIsDisabled();
  }

  // PID
  private SparkMaxPIDController m_extensionPIDController = m_extensionMotor.getPIDController();
  static double kPExtension = .05;
  static double kIExtension = 0;
  static double kDExtension = 0;
  static double kFFExtension = 0;

  static double kMaxOutputExtension = 0.85;
  static double kMinOutputExtension = -0.75;

  public boolean ExtensionRetractionLimitHit() {
    return isRetractionLimitHit();
  }

  public void disableExtensionLimitSwitches() {
    if (extensionLimitSwitchIsDisabled) {
      m_extensionLimit.enableLimitSwitch(false);
      m_retractionLimit.enableLimitSwitch(false);
    }
    ;
  }

  public Extension() {

    m_extensionMotor.restoreFactoryDefaults();

    m_extensionMotor.setIdleMode(IdleMode.kBrake);
    m_extensionMotor.setSmartCurrentLimit((SubsystemMotorConstants.kMotorCurrentLimit));
    m_extensionMotor.setOpenLoopRampRate(ExtensionConstants.kExtensionRampRate);

    m_extensionPIDController.setFeedbackDevice(m_extensionEncoder);
    m_extensionPIDController.setP(kPExtension, 0);
    m_extensionPIDController.setI(kIExtension, 0);
    m_extensionPIDController.setD(kDExtension, 0);
    m_extensionPIDController.setFF(kFFExtension, 0);
    m_extensionPIDController.setOutputRange(kMinOutputExtension, kMaxOutputExtension);

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

    addExtensionLimitSwitchDisableToDashboard();

  }

  /** The log method puts interesting information to the SmartDashboard. */
  public void log() {

    SmartDashboard.putNumber("Extension Current position", getCurrentExtensionPosition());

    if (TUNING_MODE) {
      // SmartDashboard.putBoolean("Fully Extended", m_extensionLimit.isPressed());
      // SmartDashboard.putBoolean("Fully Retracted", m_retractionLimit.isPressed());
      // SmartDashboard.putBoolean("Extension Travel Position",
      // ExtensionIsInTravelPosition());
      // SmartDashboard.putBoolean("Extension Intake Position",
      // ExtensionIsInIntakePosition());
      addPIDToDashboard();
      readExtensionPIDTuningFromDashboard();

      if (ExtensionIsInHighScoringPosition()) {
        System.out.println("Extension in HIGH SCORING position");
      }
    }
  }

  public void displayDisableExtensionLimitSwitch() {
    // enable/disable limit switches based on value read from SmartDashboard
    m_extensionLimit.enableLimitSwitch(SmartDashboard.getBoolean("Extension Limit Switch Enabled", true));
    m_retractionLimit.enableLimitSwitch(SmartDashboard.getBoolean("Retraction Limit Switch Enabled", true));

  }

  public void addExtensionLimitSwitchDisableToDashboard() {
    SmartDashboard.getBoolean("Extension Limit Switch Enabled", m_extensionLimit.isLimitSwitchEnabled());
    SmartDashboard.getBoolean("Retraction Limit Switch Enabled", m_retractionLimit.isLimitSwitchEnabled());
  }

  private void addPIDToDashboard() {
    // Display PID Extension coefficients on SmartDashboard
    SmartDashboard.putNumber("Extension P Gain", kPExtension);
    SmartDashboard.putNumber("Extension I Gain", kIExtension);
    SmartDashboard.putNumber("Extension D Gain", kDExtension);
    SmartDashboard.putNumber("Extension Max Output", kMaxOutputExtension);
    SmartDashboard.putNumber("Extension Min Output", kMinOutputExtension);

  }

  private void readExtensionPIDTuningFromDashboard() {

    // Read PID Coefficients from SmartDashboard
    double pExtension = SmartDashboard.getNumber("Extension P Gain", 0);
    double iExtension = SmartDashboard.getNumber("Extension I Gain", 0);
    double dExtension = SmartDashboard.getNumber("Extension D Gain", 0);
    double maxExtension = SmartDashboard.getNumber("Extension Max Output", 0);
    double minExtension = SmartDashboard.getNumber("Extension Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller. Make sure to use the PID Extension slot
    if ((pExtension != kPExtension)) {
      m_extensionPIDController.setP(pExtension);
      kPExtension = pExtension;
    }
    if ((iExtension != kIExtension)) {
      m_extensionPIDController.setI(iExtension);
      kIExtension = iExtension;
    }
    if ((dExtension != kDExtension)) {
      m_extensionPIDController.setD(dExtension);
      kDExtension = dExtension;
    }

    if ((maxExtension != kMaxOutputExtension) || (minExtension != kMinOutputExtension)) {
      m_extensionPIDController.setOutputRange(minExtension, maxExtension);
      kMinOutputExtension = minExtension;
      kMaxOutputExtension = maxExtension;
    }
  }

  public void logPositionsReached() {
    if (ExtensionIsInDropOffReturnPosition()) {
      System.out.println("Extension in DROP OFF RETURN position");
    }
    if (ExtensionIsInHighCubeShootPosition()) {
      System.out.println("Extension in HIGH CUBE SHOOT position");
    }
    if (ExtensionIsInHighScoringPosition()) {
      System.out.println("Extension in HIGH SCORING position");
    }
    if (ExtensionIsInIntakePosition()) {
      System.out.println("Extension in INTAKE position");
    }
    if (ExtensionIsInMidCubeShootPosition()) {
      System.out.println("Extension in MID CUBE SHOOT position");
    }
    if (ExtensionIsInReleasePosition()) {
      System.out.println("Extension in RELEASE position");
    }
  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();
    ExtensionRetractionLimitHit();
    displayDisableExtensionLimitSwitch();
  }

  /** Resets the Extension encoder to currently read a position of 0. */
  public void reset() {
    m_extensionEncoder.setPosition(0);
  }

  // Reset the Extension Encoder when the Retraction Limit is pressed
  public boolean isRetractionLimitHit() {
    return m_retractionLimit.isPressed();
  }

  public void resetExtensionEncoderAtRetractionLimit() {
    m_extensionEncoder.setPosition(1);
  }

  /** ELEVATOR Extension **/
  public void moveExtension(double speed) {
    m_extensionMotor.set(speed);
  }

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

  // Maintain Position
  public void keepPosition(double position) {
    m_extensionPIDController.setReference(position, ControlType.kPosition);
    if (TUNING_MODE) {
      SmartDashboard.putNumber("Desired Extension position", position);
      System.out.println("Keep EXTENSION " + position);
    }
  }

  // Maintain Position Inches
  public void keepPositionInches(double positionInches) {
    // set position in inches, convert to encoder value
    double position;
    position = positionInches * ExtensionConstants.kExtensionRevolutionsPerInch + 1;

    m_extensionPIDController.setReference(position, ControlType.kPosition);
    if (TUNING_MODE) {
      SmartDashboard.putNumber("Desired Extension position", position);
      System.out.println("Keep EXTENSION " + position);
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

  public boolean ExtensionIsInCubeShootPosition() {
    return m_extensionEncoder.getPosition() < ExtensionConstants.kExtensionPositionCubeShooter
        + ExtensionConstants.kExtensionPositionTolerance
        && m_extensionEncoder.getPosition() > ExtensionConstants.kExtensionPositionCubeShooter
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
