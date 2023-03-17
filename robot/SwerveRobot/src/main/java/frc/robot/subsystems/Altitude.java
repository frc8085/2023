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
import frc.robot.Constants.ExtensionConstants;

import static frc.robot.Constants.AltitudeConstants;
import static frc.robot.Constants.SubsystemMotorConstants;

public class Altitude extends SubsystemBase {
  private boolean TUNING_MODE = false;

  private Extension m_extension;
  /** Creates a new Altitude. */

  // Altitude motors
  private final CANSparkMax m_altitudeMotor = new CANSparkMax(AltitudeConstants.kAltitudeMotorPort,
      MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder m_altitudeEncoder = m_altitudeMotor.getEncoder();

  // Limit Switches
  private SparkMaxLimitSwitch m_altitudeTopLimit;
  private SparkMaxLimitSwitch m_altitudeBottomLimit;

  // PID for raising
  private SparkMaxPIDController m_altitudeRaisePIDController = m_altitudeMotor.getPIDController();
  static double kPAltitudeRaise = 1;
  static double kIAltitudeRaise = 0.0001;
  static double kDAltitudeRaise = 0.1;
  // static double kIzAltitudeRaise = 0;
  static double kFFAltitudeRaise = 0;
  static double kMaxOutputAltitudeRaise = 9;
  static double kMinOutputAltitudeRaise = -.9;

  // PID for lowering
  private SparkMaxPIDController m_altitudeLowerPIDController = m_altitudeMotor.getPIDController();
  static double kPAltitudeLower = 1;
  static double kIAltitudeLower = 0.0001;
  static double kDAltitudeLower = 0.1;
  static double kFFAltitudeLower = 0;
  static double kMaxOutputAltitudeLower = 9;
  static double kMinOutputAltitudeLower = -.9;

  // When INTAKE, extension only in intake position
  // When TRAVEL, extenion only in travel
  public void enforceSafeExtensions() {
    if (AltitudeIsInIntakePosition()) {
      m_extension.keepPosition(ExtensionConstants.kExtensionPositionIntakeOut);
    }

    // if (AltitudeIsInTravelPosition()) {
    // m_extension.keepPosition(ExtensionConstants.kExtensionPositionFullyRetracted);
    // }

  }

  public Altitude(Extension Extension) {
    m_extension = Extension;
    m_altitudeMotor.setIdleMode(IdleMode.kBrake);
    m_altitudeMotor.setSmartCurrentLimit(SubsystemMotorConstants.kMotorCurrentLimit);
    m_altitudeMotor.setOpenLoopRampRate(AltitudeConstants.kAltitudeRampRate);

    m_altitudeRaisePIDController.setFeedbackDevice(m_altitudeEncoder);
    m_altitudeRaisePIDController.setP(kPAltitudeRaise, 0);
    m_altitudeRaisePIDController.setI(kIAltitudeRaise, 0);
    m_altitudeRaisePIDController.setD(kDAltitudeRaise, 0);
    m_altitudeRaisePIDController.setFF(kFFAltitudeRaise, 0);
    m_altitudeRaisePIDController.setOutputRange(kMinOutputAltitudeRaise, kMaxOutputAltitudeRaise);

    m_altitudeLowerPIDController.setFeedbackDevice(m_altitudeEncoder);
    m_altitudeLowerPIDController.setP(kPAltitudeLower, 0);
    m_altitudeLowerPIDController.setI(kIAltitudeLower, 0);
    m_altitudeLowerPIDController.setD(kDAltitudeLower, 0);
    m_altitudeLowerPIDController.setFF(kFFAltitudeLower, 0);
    m_altitudeLowerPIDController.setOutputRange(kMinOutputAltitudeLower, kMaxOutputAltitudeLower);

    // TODO. What should these values be?
    m_altitudeRaisePIDController.setSmartMotionMaxAccel(0.5, 0);
    m_altitudeRaisePIDController.setSmartMotionMaxVelocity(0.5, 0);

    m_altitudeLowerPIDController.setSmartMotionMaxAccel(0.5, 0);
    m_altitudeLowerPIDController.setSmartMotionMaxVelocity(0.5, 0);

    /**
     * A SparkMaxLimitSwitch object is constructed using the getForwardLimitSwitch()
     * or
     * getReverseLimitSwitch() method on an existing CANSparkMax object, depending
     * on which direction you would like to limit
     * 
     * Limit switches can be configured to one of two polarities:
     * com.revrobotics.SparkMaxLimitSwitch.SparkMaxLimitSwitch.Type.kNormallyOpen
     * com.revrobotics.SparkMaxLimitSwitch.SparkMaxLimitSwitch.Type.kNormallyClosed
     */
    m_altitudeTopLimit = m_altitudeMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_altitudeBottomLimit = m_altitudeMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_altitudeMotor.burnFlash();

    // If we're fine-tuning PID Constants, the display them on the dashboard
    if (TUNING_MODE) {
      addPIDToDashboard();
      addTuningtoDashboard();
    }

  }

  /** The log method puts interesting information to the SmartDashboard. */
  public void log() {

    if (TUNING_MODE) {
      // readPIDTuningFromDashboard();
      // readTuningFromDashboard();

      SmartDashboard.putNumber("Altitude Raw encoder read", m_altitudeEncoder.getPosition());
      SmartDashboard.putBoolean("Altitude at Top Position", m_altitudeTopLimit.isPressed());
      SmartDashboard.putBoolean("Altitude at Bottom Position", m_altitudeBottomLimit.isPressed());
      SmartDashboard.putBoolean("Altitude is in Travel Position", AltitudeIsInTravelPosition());
      SmartDashboard.putNumber("Current altitude", getCurrentAltitude());
    }

  }

  private void addPIDToDashboard() {
    // Display PID Raise coefficients on SmartDashboard
    SmartDashboard.putNumber("AltitudeRaise P Gain", kPAltitudeRaise);
    SmartDashboard.putNumber("AltitudeRaise I Gain", kIAltitudeRaise);
    SmartDashboard.putNumber("AltitudeRaise D Gain", kDAltitudeRaise);
    // SmartDashboard.putNumber("AltitudeRaise I Zone", kIzAltitudeRaise);
    SmartDashboard.putNumber("AltitudeRaise Feed Forward", kFFAltitudeRaise);
    SmartDashboard.putNumber("AltitudeRaise Max Output", kMaxOutputAltitudeRaise);
    SmartDashboard.putNumber("AltitudeRaise Min Output", kMinOutputAltitudeRaise);
    SmartDashboard.putNumber("AltitudeRaise Set Rotations", 0);

    // Display PID Lower coefficients on SmartDashboard
    SmartDashboard.putNumber("AltitudeLower P Gain", kPAltitudeLower);
    SmartDashboard.putNumber("AltitudeLower I Gain", kIAltitudeLower);
    SmartDashboard.putNumber("AltitudeLower D Gain", kDAltitudeLower);
    // SmartDashboard.putNumber("AltitudeLower I Zone", kIzAltitudeLower);
    SmartDashboard.putNumber("AltitudeLower Feed Forward", kFFAltitudeLower);
    SmartDashboard.putNumber("AltitudeLower Max Output", kMaxOutputAltitudeLower);
    SmartDashboard.putNumber("AltitudeLower Min Output", kMinOutputAltitudeLower);
    SmartDashboard.putNumber("AltitudeLower Set Rotations", 0);
  }

  private void addTuningtoDashboard() {
    // Substation Constants
    SmartDashboard.putNumber("Single Substation Altitude", AltitudeConstants.kAltitudeSingleSubstationPosition);

    // High Drop Off Adjustment
    SmartDashboard.putNumber("High Drop Off Position", AltitudeConstants.kAltitudeHighDropOffPosition);
    SmartDashboard.putNumber("High Drop Off Final Position", AltitudeConstants.kAltitudeHighDropOffFinalPosition);

    // Mid Drop Off Adjustment
    SmartDashboard.putNumber("Mid Drop Off Position", AltitudeConstants.kAltitudeMidDropOffPosition);
    SmartDashboard.putNumber("Mid Drop Off Final Position", AltitudeConstants.kAltitudeMidDropOffFinalPosition);

  }

  private void readTuningFromDashboard() {
    double SingleSubstationAltitude = SmartDashboard.getNumber("Single Substation Altitude",
        AltitudeConstants.kAltitudeSingleSubstationPosition);
    double HighDropOffAltitude = SmartDashboard.getNumber("High Drop Off Position",
        AltitudeConstants.kAltitudeHighDropOffPosition);
    double HighDropOffFinalAltitude = SmartDashboard.getNumber("High Drop Off Final Position",
        AltitudeConstants.kAltitudeHighDropOffFinalPosition);
    double MidDropOffAltitude = SmartDashboard.getNumber("Mid Drop Off Position",
        AltitudeConstants.kAltitudeMidDropOffPosition);
    double MidDropOffFinalAltitude = SmartDashboard.getNumber("Mid Drop Off Final Position",
        AltitudeConstants.kAltitudeMidDropOffFinalPosition);

    if ((SingleSubstationAltitude != AltitudeConstants.kAltitudeSingleSubstationPosition)) {
      keepPosition(SingleSubstationAltitude);
      AltitudeConstants.kAltitudeSingleSubstationPosition = SingleSubstationAltitude;
    }
    if ((HighDropOffAltitude != AltitudeConstants.kAltitudeHighDropOffPosition)) {
      keepPosition(HighDropOffAltitude);
      AltitudeConstants.kAltitudeHighDropOffPosition = SingleSubstationAltitude;
    }
    if ((HighDropOffFinalAltitude != AltitudeConstants.kAltitudeHighDropOffFinalPosition)) {
      keepPosition(HighDropOffFinalAltitude);
      AltitudeConstants.kAltitudeHighDropOffFinalPosition = SingleSubstationAltitude;
    }
    if ((MidDropOffAltitude != AltitudeConstants.kAltitudeMidDropOffPosition)) {
      keepPosition(MidDropOffAltitude);
      AltitudeConstants.kAltitudeMidDropOffPosition = SingleSubstationAltitude;
    }
    if ((MidDropOffFinalAltitude != AltitudeConstants.kAltitudeMidDropOffFinalPosition)) {
      keepPosition(MidDropOffFinalAltitude);
      AltitudeConstants.kAltitudeMidDropOffFinalPosition = SingleSubstationAltitude;
    }

  }

  private void readRaisePIDTuningFromDashboard() {

    // Read PID Coefficients from SmartDashboard
    double pAltitudeRaise = SmartDashboard.getNumber("AltitudeRaise P Gain", 0);
    double iAltitudeRaise = SmartDashboard.getNumber("AltitudeRaise I Gain", 0);
    double dAltitudeRaise = SmartDashboard.getNumber("AltitudeRaise D Gain", 0);
    // double izAltitudeRaise = SmartDashboard.getNumber("AltitudeRaise I Zone", 0);
    double ffAltitudeRaise = SmartDashboard.getNumber("AltitudeRaise Feed Forward", 0);
    double maxAltitudeRaise = SmartDashboard.getNumber("AltitudeRaise Max Output", 0);
    double minAltitudeRaise = SmartDashboard.getNumber("AltitudeRaise Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((pAltitudeRaise != kPAltitudeRaise)) {
      m_altitudeRaisePIDController.setP(pAltitudeRaise);
      kPAltitudeRaise = pAltitudeRaise;
    }
    if ((iAltitudeRaise != kIAltitudeRaise)) {
      m_altitudeRaisePIDController.setI(iAltitudeRaise);
      kIAltitudeRaise = iAltitudeRaise;
    }
    if ((dAltitudeRaise != kDAltitudeRaise)) {
      m_altitudeRaisePIDController.setD(dAltitudeRaise);
      kDAltitudeRaise = dAltitudeRaise;
    }

    /**
     * if ((izAltitude != kIzAltitude)) {
     * m_altitudePIDController.setIZone(izAltitude);
     * kIzAltitude = izAltitude;
     * }
     */

    if ((ffAltitudeRaise != kFFAltitudeRaise)) {
      m_altitudeRaisePIDController.setFF(ffAltitudeRaise);
      kFFAltitudeRaise = ffAltitudeRaise;
    }

    if ((maxAltitudeRaise != kMaxOutputAltitudeRaise) || (minAltitudeRaise != kMinOutputAltitudeRaise)) {
      m_altitudeRaisePIDController.setOutputRange(minAltitudeRaise, maxAltitudeRaise);
      kMinOutputAltitudeRaise = minAltitudeRaise;
      kMaxOutputAltitudeRaise = maxAltitudeRaise;
    }
  }

  private void readLowerPIDTuningFromDashboard() {

    // Read PID Coefficients from SmartDashboard
    double pAltitudeLower = SmartDashboard.getNumber("AltitudeLower P Gain", 0);
    double iAltitudeLower = SmartDashboard.getNumber("AltitudeLower I Gain", 0);
    double dAltitudeLower = SmartDashboard.getNumber("AltitudeLower D Gain", 0);
    // double izAltitudeLower = SmartDashboard.getNumber("AltitudeLower I Zone", 0);
    double ffAltitudeLower = SmartDashboard.getNumber("AltitudeLower Feed Forward", 0);
    double maxAltitudeLower = SmartDashboard.getNumber("AltitudeLower Max Output", 0);
    double minAltitudeLower = SmartDashboard.getNumber("AltitudeLower Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((pAltitudeLower != kPAltitudeLower)) {
      m_altitudeLowerPIDController.setP(pAltitudeLower);
      kPAltitudeLower = pAltitudeLower;
    }
    if ((iAltitudeLower != kIAltitudeLower)) {
      m_altitudeLowerPIDController.setI(iAltitudeLower);
      kIAltitudeLower = iAltitudeLower;
    }
    if ((dAltitudeLower != kDAltitudeLower)) {
      m_altitudeLowerPIDController.setD(dAltitudeLower);
      kDAltitudeLower = dAltitudeLower;
    }

    if ((ffAltitudeLower != kFFAltitudeLower)) {
      m_altitudeLowerPIDController.setFF(ffAltitudeLower);
      kFFAltitudeLower = ffAltitudeLower;
    }

    if ((maxAltitudeLower != kMaxOutputAltitudeLower) || (minAltitudeLower != kMinOutputAltitudeLower)) {
      m_altitudeLowerPIDController.setOutputRange(minAltitudeLower, maxAltitudeLower);
      kMinOutputAltitudeLower = minAltitudeLower;
      kMaxOutputAltitudeLower = maxAltitudeLower;
    }
  }

  /** Call log method every loop. */
  @Override
  public void periodic() {

    enforceSafeExtensions();

    // resetAltitudeEncoderAtTopLimit();
    AltitudeIsInTravelPosition();
    AltitudeIsInIntakePosition();

    if (TUNING_MODE) {
      readRaisePIDTuningFromDashboard();
      readLowerPIDTuningFromDashboard();
      readTuningFromDashboard();
      log();

    }

  }

  /** Resets the Altitude encoder to currently read a position of 0. */
  public void reset() {
    m_altitudeEncoder.setPosition(0);
  }

  // Reset the altitude Encoder when the top limit is pressed
  public boolean isAltitudeTopLimitHit() {
    return m_altitudeTopLimit.isPressed() == true;
  }

  public void resetAltitudeEncoderAtTopLimit() {
    m_altitudeEncoder.setPosition(-0.1);
  };

  /** ALTITUDE **/
  // Run the Altitude motor forward
  public void raiseAltitude() {
    m_altitudeMotor.set(AltitudeConstants.kAltitudeRaiseSpeed);
  }

  // Run the Altitude motor in reverse
  public void lowerAltitude() {
    m_altitudeMotor.set(-AltitudeConstants.kAltitudeLowerSpeed);
  }

  // Stop the Altitude
  public void stopAltitude() {
    m_altitudeMotor.set(0);
  }

  // Returns the current altitude
  public double getCurrentAltitude() {
    return m_altitudeEncoder.getPosition();
  }

  /**
   * Move to and stay at a certain altitude
   * Use the Raising PID coefficients if we are going up
   * Otherwise use the Lowering PID coefficients
   * 
   * @param positionAltitude The desired altitude
   **/

  public void keepPosition(double positionAltitude) {
    boolean raising = getCurrentAltitude() <= positionAltitude;

    if (raising) {
      m_altitudeRaisePIDController.setReference(positionAltitude, ControlType.kPosition);
    } else {
      m_altitudeLowerPIDController.setReference(positionAltitude, ControlType.kPosition);
    }

    SmartDashboard.putNumber("Altitude Desired position", positionAltitude);
  }

  // Tell Us if Altitude as At Set Positions

  public boolean AltitudeIsInTravelPosition() {
    return m_altitudeEncoder
        .getPosition() > (AltitudeConstants.kAltitudeTravelPosition
            - AltitudeConstants.kAltitudePositionTolerance);
  }

  public boolean AltitudeIsInIntakePosition() {
    return m_altitudeEncoder.getPosition() < AltitudeConstants.kAltitudeIntakePosition
        + AltitudeConstants.kAltitudePositionTolerance;

  }

  public boolean AltitudeIsInScoringPosition() {
    return m_altitudeEncoder.getPosition() < AltitudeConstants.kAltitudeHighDropOffPosition
        + AltitudeConstants.kAltitudePositionTolerance &&
        m_altitudeEncoder.getPosition() > AltitudeConstants.kAltitudeMidDropOffPosition
            - AltitudeConstants.kAltitudePositionTolerance;
  };

  public boolean AltitudeIsInHighDropOffFinalPosition() {
    return m_altitudeEncoder.getPosition() < AltitudeConstants.kAltitudeHighDropOffFinalPosition
        + AltitudeConstants.kAltitudePositionTolerance &&
        m_altitudeEncoder.getPosition() > AltitudeConstants.kAltitudeHighDropOffFinalPosition
            - AltitudeConstants.kAltitudePositionTolerance;
  };

  public boolean AltitudeIsInMidDropOffFinalPosition() {
    return m_altitudeEncoder.getPosition() < AltitudeConstants.kAltitudeMidDropOffFinalPosition
        + AltitudeConstants.kAltitudePositionTolerance &&
        m_altitudeEncoder.getPosition() > AltitudeConstants.kAltitudeMidDropOffFinalPosition
            - AltitudeConstants.kAltitudePositionTolerance;
  };

  public boolean AltitudeIsInHighCubeShootPosition() {
    return m_altitudeEncoder.getPosition() < AltitudeConstants.kAltitudeHighCubeShootPosition
        + AltitudeConstants.kAltitudePositionTolerance &&
        m_altitudeEncoder.getPosition() > AltitudeConstants.kAltitudeHighCubeShootPosition
            - AltitudeConstants.kAltitudePositionTolerance;
  };

}
