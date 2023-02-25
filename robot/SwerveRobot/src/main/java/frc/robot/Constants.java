// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class IntakeCoverConstants {
    public static final int[] kIntakeCoverSolenoidPorts = new int[] { 6, 7 };
  }

  public static final class SubsystemMotorConstants {
    public static final int kMotorCurrentLimit = 30;
    public static final int kMotorCurrentLimit550 = 30;
  }

  public static final class IntakeConstants {
    public static final int kIntakePort = 6;

    // {kRampRate} = Time in seconds to go from 0 to full throttle.
    public static double kRampRate = 0;

    public static final double kIntakeToleranceRPMPercent = 0.10;

    // Intake off speed
    public static final int kIntakeOffSpeed = 0;

    // Intake Cone Speeds
    public static final int kIntakeConeSpeed = 2000;
    public static final int kEjectConeSpeed = -500;

    // Intake Cube speeds
    public static final int kIntakeCubeSpeed = 1500;
    public static final int kEjectCubeSpeed = -2000;
    public static final int kHoldCubeSpeed = 500;

    // map of modes

    public static final int[] kIntakeTargetRPM = new int[] {
        kIntakeOffSpeed, // 0, kIntakeOffSpeed
        kIntakeConeSpeed, // 1, kIntakeConeSpeed
        kEjectConeSpeed, // 2, kEjectConeSpeed
        kIntakeCubeSpeed, // 3, kIntakeCubeSpeed
        kEjectCubeSpeed, // 4, kEjectCubeSpeed
    };

    /** Intake off index in { @see IntakeConstants.kIntakeTargetRPM } */
    public static final int kCargoNone = 0;

    /** Cone Indices in { @see IntakeConstants.kIntakeTargetRPM } */
    public static final int kCargoConeIntake = 1;
    public static final int kCargoConeEject = 2;

    /** Cube Indices in { @see IntakeConstants.kIntakeTargetRPM } */
    public static final int kCargoCubeIntake = 3;
    public static final int kCargoCubeEject = 4;

  }

  public static final class IntakeNoPIDConstants {
    // Intake Cone Power
    public static final double kIntakeConePower = 0.8;
    public static final double kEjectConePower = 0.5;

    // Intake Cube Power
    public static final double kIntakeCubePower = 0.6;
    public static final double kEjectCubePower = 1;
    public static final double kIntakeHoldCubePower = 0.2;

    public static final double kEjectWaitTime = 2;
  }

  public static final class ExtensionConstants {
    public static int kExtensionMotorPort = 7;
    public static double kExtensionSpeed = 0.5;
    public static double kExtensionRampRate = 0;
    public static double kExtensionPositionTolerance = 1;

    public static final double kMaxExtensionSpeedMetersPerSecond = 0.25;

    // Range for safe Travel Extension <20
    public static double kExtensionSafeMax = 10;

    // encoder readings of Extension position as of 2.14.2023
    public static double kExtensionPositionFullyRetracted = 1;
    public static double kExtensionPositionIntakeOut = 39;
    public static double kExtensionPositionMidDropOff = 83;
    public static double kExtensionPositionHighDropOff = 135;

  }

  public static final class AltitudeConstants {
    public static int kAltitudeMotorPort = 5;
    public static double kAltitudeStopSpeed = 0;
    public static double kAltitudeSpeed = 0.2;
    public static double kAltitudeRampRate = 1;
    public static double kAltitudePositionTolerance = 1;

    public static final double kMaxAltitudeSpeedMetersPerSecond = .05;
    public static final double kMaxLimitedAltitudeSpeedMetersPerSecond = .01;

    // Range for safe Travel altitude > -0.5
    public static double kAltitudeSafeMin = -0.5;

    // encoder readings of altitude as of 2.14.2023
    // Encoder at Top Position
    public static double kAltitudeTravelPosition = -0.15;
    // Encoder at Mid Position
    public static double kAltitudeDropOffPosition = -2.5;
    // Encoder at Bottom Position
    public static double kAltitudeIntakePosition = -5.2;
    // Encoder at Shelf Position
    public static double kAltitudeShelfPosition = -2.1;
    public static double kAltitudeError = 0.05;

    // estimates of angles for altitude
    public static double kAltitudeTravelPositionAngle = 0;
    public static double kAltitudeDropOffPositionAngle = -40;
    public static double kAltitudeIntakePositionAngle = -80;
    public static double kAltitudeToleranceAngle = 2;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kFixedMidSpeedLimit = 0.30;
    // maxSpeedMetersPerSecond default = 4.8
    public static final double kMaxSpeedMetersPerSecond = 3.5;
    // reduced this from 2 * Math.PI to slow down rotation
    public static final double kMaxAngularSpeed = .5 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 2;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 8;
    public static final int kRearLeftTurningCanId = 9;
    public static final int kFrontRightTurningCanId = 10;
    public static final int kRearRightTurningCanId = 11;

    // GYRO CONSTANTS
    public static final int kGyroDeviceNumber = 15;
    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
    public static final double kMagnitudeDeadband = 0.05;
    public static final double kDirectionSlewRate = 2.4; // radians per second
    public static final double kMagnitudeSlewRate = 3.6; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 4.0; // percent per second (1 = 100%)
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = 0.25 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 0.25 * Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
