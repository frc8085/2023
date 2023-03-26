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
import frc.robot.Constants.TuningModeConstants;

import static frc.robot.Constants.AltitudeConstants;
import static frc.robot.Constants.SubsystemMotorConstants;

import java.util.function.BooleanSupplier;

public class Altitude extends SubsystemBase {
  private boolean TUNING_MODE = TuningModeConstants.kAltitudeTuning;

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

  private SparkMaxPIDController m_altitudePIDController = m_altitudeMotor.getPIDController();

  // Raise PID coefficients
  static int kRaisePIDSlot = 0;
  static double kPRaise = 1;
  static double kIRaise = 0.0001;
  static double kDRaise = 0.1;
  // static double kIzRaise = 0;
  static double kFFRaise = 0;
  static double kMaxOutputRaise = 1;
  static double kMinOutputRaise = -1;

  // Lower PID coefficients
  static int kLowerPIDSlot = 1;
  static double kPLower = .5;
  static double kILower = 0.0001;
  static double kDLower = 0.1;
  static double kFFLower = 0;
  static double kMaxOutputLower = .9;
  static double kMinOutputLower = -.9;

  // When INTAKE, extension only in intake position
  // When TRAVEL, extenion only in travel
  public void enforceSafeExtensions() {
    if (AltitudeIsInIntakePosition()) {
      m_extension.keepPositionInches(ExtensionConstants.kExtensionPositionInchesIntakeOut);
    }

    // if (AltitudeIsInTravelPosition()) {
    // m_extension.keepPosition(ExtensionConstants.kExtensionPositionInchesFullyRetracted);
    // }

  }

  public Altitude(Extension Extension) {
    m_extension = Extension;
    m_altitudeMotor.setIdleMode(IdleMode.kBrake);
    m_altitudeMotor.setSmartCurrentLimit(SubsystemMotorConstants.kMotorCurrentLimit);
    m_altitudeMotor.setOpenLoopRampRate(AltitudeConstants.kAltitudeRampRate);

    m_altitudePIDController.setFeedbackDevice(m_altitudeEncoder);

    // ** RAISE PID: Use Raise coefficients and slotID for Raise PID */
    m_altitudePIDController.setP(kPRaise, kRaisePIDSlot);
    m_altitudePIDController.setI(kIRaise, kRaisePIDSlot);
    m_altitudePIDController.setD(kDRaise, kRaisePIDSlot);
    m_altitudePIDController.setFF(kFFRaise, kRaisePIDSlot);
    m_altitudePIDController.setOutputRange(kMinOutputRaise, kMaxOutputRaise, kRaisePIDSlot);
    m_altitudePIDController.setSmartMotionMaxAccel(0.5, kRaisePIDSlot);
    m_altitudePIDController.setSmartMotionMaxVelocity(0.5, kRaisePIDSlot);
    // TODO. What should these values be?
    m_altitudePIDController.setSmartMotionMaxAccel(0.5, kLowerPIDSlot);
    m_altitudePIDController.setSmartMotionMaxVelocity(0.5, kLowerPIDSlot);

    // ** LOWER PID: Use Lower coefficients and slotID for Lower PID
    m_altitudePIDController.setP(kPLower, kLowerPIDSlot);
    m_altitudePIDController.setI(kILower, kLowerPIDSlot);
    m_altitudePIDController.setD(kDLower, kLowerPIDSlot);
    m_altitudePIDController.setFF(kFFLower, kLowerPIDSlot);
    m_altitudePIDController.setOutputRange(kMinOutputLower, kMaxOutputLower, kLowerPIDSlot);
    // TODO. What should these values be?
    m_altitudePIDController.setSmartMotionMaxAccel(0.5, kLowerPIDSlot);
    m_altitudePIDController.setSmartMotionMaxVelocity(0.5, kLowerPIDSlot);

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
    SmartDashboard.putNumber("Altitude: Current reading", getCurrentAltitude());

    if (TUNING_MODE) {
      // SmartDashboard.putBoolean("Altitude at Top Position",
      // m_altitudeTopLimit.isPressed());
      // SmartDashboard.putBoolean("Altitude at Bottom Position",
      // m_altitudeBottomLimit.isPressed());
      // SmartDashboard.putBoolean("Altitude is in Travel Position",
      // AltitudeIsInTravelPosition());

      readRaisePIDTuningFromDashboard();
      readLowerPIDTuningFromDashboard();
      // readTuningFromDashboard();

    }

  }

  public void logPositionsReached() {
    if (AltitudeIsInHighCubeShootPosition()) {
      System.out.println("Altitude in HIGH CUBE SHOOT position");
    }
    if (AltitudeIsInHighDropOffFinalPosition()) {
      System.out.println("Altitude in HIGH DROP OFF FINAL position");
    }
    if (AltitudeIsInIntakePosition()) {
      System.out.println("Altitude in INTAKE position");
    }
    if (AltitudeIsInMidDropOffFinalPosition()) {
      System.out.println("Altitude in MID DROP OFF FINAL position");
    }
    if (AltitudeIsInScoringPosition()) {
      System.out.println("Altitude in SCORING position");
    }
    if (AltitudeIsInTravelPosition()) {
      System.out.println("Altitude in TRAVEL position");
    }
  }

  private void addPIDToDashboard() {
    // Display PID Raise coefficients on SmartDashboard
    SmartDashboard.putNumber("Raise P Gain", kPRaise);
    SmartDashboard.putNumber("Raise I Gain", kIRaise);
    SmartDashboard.putNumber("Raise D Gain", kDRaise);
    // SmartDashboard.putNumber("Raise I Zone", kIzRaise);
    SmartDashboard.putNumber("Raise Feed Forward", kFFRaise);
    SmartDashboard.putNumber("Raise Max Output", kMaxOutputRaise);
    SmartDashboard.putNumber("Raise Min Output", kMinOutputRaise);
    SmartDashboard.putNumber("Raise Set Rotations", 0);

    // Display PID Lower coefficients on SmartDashboard
    SmartDashboard.putNumber("Lower P Gain", kPLower);
    SmartDashboard.putNumber("Lower I Gain", kILower);
    SmartDashboard.putNumber("Lower D Gain", kDLower);
    // SmartDashboard.putNumber("Lower I Zone", kIzLower);
    SmartDashboard.putNumber("Lower Feed Forward", kFFLower);
    SmartDashboard.putNumber("Lower Max Output", kMaxOutputLower);
    SmartDashboard.putNumber("Lower Min Output", kMinOutputLower);
    SmartDashboard.putNumber("Lower Set Rotations", 0);
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
    double pRaise = SmartDashboard.getNumber("Raise P Gain", 0);
    double iRaise = SmartDashboard.getNumber("Raise I Gain", 0);
    double dRaise = SmartDashboard.getNumber("Raise D Gain", 0);
    // double izRaise = SmartDashboard.getNumber("Raise I Zone", 0);
    double ffRaise = SmartDashboard.getNumber("Raise Feed Forward", 0);
    double maxRaise = SmartDashboard.getNumber("Raise Max Output", 0);
    double minRaise = SmartDashboard.getNumber("Raise Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller. Make sure to use the PID Raise slot
    if ((pRaise != kPRaise)) {
      m_altitudePIDController.setP(pRaise, kRaisePIDSlot);
      kPRaise = pRaise;
    }
    if ((iRaise != kIRaise)) {
      m_altitudePIDController.setI(iRaise, kRaisePIDSlot);
      kIRaise = iRaise;
    }
    if ((dRaise != kDRaise)) {
      m_altitudePIDController.setD(dRaise, kRaisePIDSlot);
      kDRaise = dRaise;
    }

    if ((ffRaise != kFFRaise)) {
      m_altitudePIDController.setFF(ffRaise, kRaisePIDSlot);
      kFFRaise = ffRaise;
    }

    if ((maxRaise != kMaxOutputRaise) || (minRaise != kMinOutputRaise)) {
      m_altitudePIDController.setOutputRange(minRaise, maxRaise, kRaisePIDSlot);
      kMinOutputRaise = minRaise;
      kMaxOutputRaise = maxRaise;
    }
  }

  private void readLowerPIDTuningFromDashboard() {

    // Read PID Coefficients from SmartDashboard
    double pLower = SmartDashboard.getNumber("Lower P Gain", 0);
    double iLower = SmartDashboard.getNumber("Lower I Gain", 0);
    double dLower = SmartDashboard.getNumber("Lower D Gain", 0);
    // double izLower = SmartDashboard.getNumber("Lower I Zone", 0);
    double ffLower = SmartDashboard.getNumber("Lower Feed Forward", 0);
    double maxLower = SmartDashboard.getNumber("Lower Max Output", 0);
    double minLower = SmartDashboard.getNumber("Lower Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller. Make sure to use the PID Lower slot
    if ((pLower != kPLower)) {
      m_altitudePIDController.setP(pLower, kLowerPIDSlot);
      kPLower = pLower;
    }
    if ((iLower != kILower)) {
      m_altitudePIDController.setI(iLower, kLowerPIDSlot);
      kILower = iLower;
    }
    if ((dLower != kDLower)) {
      m_altitudePIDController.setD(dLower, kLowerPIDSlot);
      kDLower = dLower;
    }

    if ((ffLower != kFFLower)) {
      m_altitudePIDController.setFF(ffLower, kLowerPIDSlot);
      kFFLower = ffLower;
    }

    if ((maxLower != kMaxOutputLower) || (minLower != kMinOutputLower)) {
      m_altitudePIDController.setOutputRange(minLower, maxLower, kLowerPIDSlot);
      kMinOutputLower = minLower;
      kMaxOutputLower = maxLower;
    }
  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();

    enforceSafeExtensions();

    // resetAltitudeEncoderAtTopLimit();
    AltitudeIsInTravelPosition();
    AltitudeIsInIntakePosition();
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

  // Manually move altitude
  public void moveAltitude(double speed) {
    m_altitudeMotor.set(speed);
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
    int slotID = raising ? kRaisePIDSlot : kLowerPIDSlot;

    // Set the position using the correct SlotID for desired PID controller (raise
    // vs lower)
    m_altitudePIDController.setReference(positionAltitude, ControlType.kPosition, slotID);

    if (TUNING_MODE) {
      SmartDashboard.putString("ALTITUDE MODE", raising ? "RAISING" : "LOWERING");
      SmartDashboard.putNumber("Altitude Desired position", positionAltitude);
      System.out.println("Keep ALTITUDE " + positionAltitude);
    }
  }

  // Maintain position in degrees
  public void keepPositionDegrees(double degreesAltitude) {
    // set degrees in altitude, convert to encoder value)
    double positionAltitude;
    positionAltitude = degreesAltitude * AltitudeConstants.kAltitudeRevolutionsPerDegree - 0.1;

    boolean raising = getCurrentAltitude() <= positionAltitude;
    int slotID = raising ? kRaisePIDSlot : kLowerPIDSlot;

    // Set the position using the correct SlotID for desired PID controller (raise
    // vs lower)
    m_altitudePIDController.setReference(positionAltitude, ControlType.kPosition, slotID);

    if (TUNING_MODE) {
      SmartDashboard.putString("ALTITUDE MODE", raising ? "RAISING" : "LOWERING");
      SmartDashboard.putNumber("Altitude Desired position", positionAltitude);
      System.out.println("Keep ALTITUDE " + positionAltitude);
    }
  }

  // Tell Us if Altitude as At Set Positions

  public boolean AltitudeIsInTravelPosition() {
    return m_altitudeEncoder
        .getPosition() > (AltitudeConstants.kAltitudeTravelPosition
            - AltitudeConstants.kAltitudePositionTolerance);
  }

  public boolean AltitudeIsInSingleSubstationPosition() {
    return m_altitudeEncoder.getPosition() < AltitudeConstants.kAltitudeSingleSubstationPosition
        + AltitudeConstants.kAltitudePositionTolerance &&
        m_altitudeEncoder.getPosition() > AltitudeConstants.kAltitudeSingleSubstationPosition
            - AltitudeConstants.kAltitudePositionTolerance;
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
