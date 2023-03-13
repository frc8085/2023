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

  // PID
  private SparkMaxPIDController m_altitudePIDController = m_altitudeMotor.getPIDController();
  static double kPAltitude = 1;
  static double kIAltitude = 0.0001;
  static double kDAltitude = 0.1;
  // static double kIzAltitude = 0;
  static double kFFAltitude = 0;
  static double kMaxOutputAltitude = 9;
  static double kMinOutputAltitude = -.9;

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

    m_altitudePIDController.setFeedbackDevice(m_altitudeEncoder);
    m_altitudePIDController.setP(kPAltitude, 0);
    m_altitudePIDController.setI(kIAltitude, 0);
    m_altitudePIDController.setD(kDAltitude, 0);
    m_altitudePIDController.setFF(kFFAltitude, 0);
    m_altitudePIDController.setOutputRange(kMinOutputAltitude, kMaxOutputAltitude);

    // TODO. What should these values be?
    m_altitudePIDController.setSmartMotionMaxAccel(0.5, 0);
    m_altitudePIDController.setSmartMotionMaxVelocity(0.5, 0);

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
    // Display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Altitude P Gain", kPAltitude);
    SmartDashboard.putNumber("Altitude I Gain", kIAltitude);
    SmartDashboard.putNumber("Altitude D Gain", kDAltitude);
    // SmartDashboard.putNumber("Altitude I Zone", kIzAltitude);
    SmartDashboard.putNumber("Altitude Feed Forward", kFFAltitude);
    SmartDashboard.putNumber("Altitude Max Output", kMaxOutputAltitude);
    SmartDashboard.putNumber("Altitude Min Output", kMinOutputAltitude);
    SmartDashboard.putNumber("Altitude Set Rotations", 0);
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
      m_altitudePIDController.setP(SingleSubstationAltitude);
      AltitudeConstants.kAltitudeSingleSubstationPosition = SingleSubstationAltitude;
    }
    if ((HighDropOffAltitude != AltitudeConstants.kAltitudeHighDropOffPosition)) {
      m_altitudePIDController.setP(HighDropOffAltitude);
      AltitudeConstants.kAltitudeHighDropOffPosition = SingleSubstationAltitude;
    }
    if ((HighDropOffFinalAltitude != AltitudeConstants.kAltitudeHighDropOffFinalPosition)) {
      m_altitudePIDController.setP(HighDropOffFinalAltitude);
      AltitudeConstants.kAltitudeHighDropOffFinalPosition = SingleSubstationAltitude;
    }
    if ((MidDropOffAltitude != AltitudeConstants.kAltitudeMidDropOffPosition)) {
      m_altitudePIDController.setP(MidDropOffAltitude);
      AltitudeConstants.kAltitudeMidDropOffPosition = SingleSubstationAltitude;
    }
    if ((MidDropOffFinalAltitude != AltitudeConstants.kAltitudeMidDropOffFinalPosition)) {
      m_altitudePIDController.setP(MidDropOffFinalAltitude);
      AltitudeConstants.kAltitudeMidDropOffFinalPosition = SingleSubstationAltitude;
    }

  }

  private void readPIDTuningFromDashboard() {

    // Read PID Coefficients from SmartDashboard
    double pAltitude = SmartDashboard.getNumber("Altitude P Gain", 0);
    double iAltitude = SmartDashboard.getNumber("Altitude I Gain", 0);
    double dAltitude = SmartDashboard.getNumber("Altitude D Gain", 0);
    // double izAltitude = SmartDashboard.getNumber("Altitude I Zone", 0);
    double ffAltitude = SmartDashboard.getNumber("Altitude Feed Forward", 0);
    double maxAltitude = SmartDashboard.getNumber("Altitude Max Output", 0);
    double minAltitude = SmartDashboard.getNumber("Altitude Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((pAltitude != kPAltitude)) {
      m_altitudePIDController.setP(pAltitude);
      kPAltitude = pAltitude;
    }
    if ((iAltitude != kIAltitude)) {
      m_altitudePIDController.setI(iAltitude);
      kIAltitude = iAltitude;
    }
    if ((dAltitude != kDAltitude)) {
      m_altitudePIDController.setD(dAltitude);
      kDAltitude = dAltitude;
    }

    /**
     * if ((izAltitude != kIzAltitude)) {
     * m_altitudePIDController.setIZone(izAltitude);
     * kIzAltitude = izAltitude;
     * }
     */

    if ((ffAltitude != kFFAltitude)) {
      m_altitudePIDController.setFF(ffAltitude);
      kFFAltitude = ffAltitude;
    }

    if ((maxAltitude != kMaxOutputAltitude) || (minAltitude != kMinOutputAltitude)) {
      m_altitudePIDController.setOutputRange(minAltitude, maxAltitude);
      kMinOutputAltitude = minAltitude;
      kMaxOutputAltitude = maxAltitude;
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
      readPIDTuningFromDashboard();
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
    m_altitudeMotor.set(AltitudeConstants.kAltitudeSpeed);
  }

  // Run the Altitude motor in reverse
  public void lowerAltitude() {
    m_altitudeMotor.set(-AltitudeConstants.kAltitudeSpeed);
  }

  // Stop the Altitude
  public void stopAltitude() {
    m_altitudeMotor.set(0);
  }

  // Returns the current altitude
  public double getCurrentAltitude() {
    return m_altitudeEncoder.getPosition();
  }

  // Maintain position
  public void keepPosition(double positionAltitude) {
    m_altitudePIDController.setReference(positionAltitude, ControlType.kPosition);
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
