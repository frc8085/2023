// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

    public static final class TuningModeConstants {
        public static final boolean kAltitudeTuning = false;
        public static final boolean kExtensionTuning = false;
        public static final boolean kIntakeTuning = false;
        public static final boolean kDriveTuning = true;
        public static final boolean kLimelightTuning = true;
    }

    public static final class SubsystemMotorConstants {
        public static final int kMotorCurrentLimit = 30;
        public static final int kMotorCurrentLimit550 = 30;
    }

    public static final class IntakeConstants {
        public static final int kIntakePort = 17;

        // {kRampRate} = Time in seconds to go from 0 to full throttle.
        public static double kRampRate = 0;

        // Intake Cone Power
        public static final double kIntakeConePower = 1;
        public static final double kEjectConePower = 0.425;

        // Intake Cube Power
        public static final double kIntakeCubePower = 0.8;
        public static final double kEjectCubePower = .8;
        public static final double kShootCubePower = 1;
        public static final double kIntakeHoldCubePower = 0.15;
        public static final double kDropCubePower = 0.15;

        public static final double kEjectWaitTime = 0.5;
        public static final double kAutoEjectWaitTime = .5;

        // Intake Button Wait Time
        public static final double kIntakeSafetyPressWaitTime = 2;

    }

    public static final class ExtensionConstants {
        public static int kExtensionMotorPort = 7;
        public static double kExtensionExtendSpeed = 0.25;
        public static double kExtensionRetractSpeed = 0.25;
        public static double kExtensionRampRate = 0;
        public static double kExtensionPositionTolerance = 1;
        public static double kAutoExtensionPositionTolerance = 5;

        public static final double kMaxExtensionSpeedMetersPerSecond = 0.25;

        // Estimates, fix this once we get exact measurements
        public static final double kExtensionLengthInches = 50;
        public static final double kExtensionLengthRevolutions = 82;

        // Convert length of travel to encoder rotations, where encoder reading of 0 is
        // 0 inches and reading of 82 is 48 inches
        public static final double kExtensionRevolutionsPerInch = (kExtensionLengthRevolutions)
                / kExtensionLengthInches;

        // Range for safe Travel Extension <20
        public static double kExtensionSafeMax = 10;

        // Converted 2.14.23 Encoder readings into inches
        public static double kExtensionPositionInchesFullyRetracted = 0;
        public static double kExtensionPositionInchesIntakeOut = 13.1;
        public static double kExtensionPositionInchesMidDropOff = 29.3;
        public static double kExtensionPositionInchesHighDropOff = 44.2;
        public static double kExtensionPositionInchesAutoHighDropOff = 46.2;
        public static double kExtensionPositionInchesHighDropOffReturn = 34.1;
        // public static double kExtensionPositionInchesSingleSubstation = 6;
        public static double kExtensionPositionInchesSingleSubstation = 4;
        public static double kExtensionPositionInchesMidCubeShooter = 20.3;
        public static double kExtensionPositionInchesHighCubeShooter = 23.8;
        public static double kExtensionPositionInchesCubeShooter = 14;

        // encoder readings of Extension position as of 3.18.2023
        public static double kExtensionPositionFullyRetracted = 0;
        public static double kExtensionPositionIntakeOut = kExtensionPositionInchesIntakeOut
                * kExtensionRevolutionsPerInch;
        public static double kExtensionPositionMidDropOff = kExtensionPositionInchesMidDropOff
                * kExtensionRevolutionsPerInch;
        public static double kExtensionPositionHighDropOff = kExtensionPositionInchesHighDropOff
                * kExtensionRevolutionsPerInch;
        public static double kExtensionPositionAutoHighDropOff = kExtensionPositionInchesAutoHighDropOff
                * kExtensionRevolutionsPerInch;
        public static double kExtensionPositionHighDropOffReturn = kExtensionPositionInchesHighDropOffReturn
                * kExtensionRevolutionsPerInch;
        public static double kExtensionPositionSingleSubstation = kExtensionPositionInchesSingleSubstation
                * kExtensionRevolutionsPerInch;
        public static double kExtensionPositionMidCubeShooter = kExtensionPositionInchesMidCubeShooter
                * kExtensionRevolutionsPerInch;
        public static double kExtensionPositionHighCubeShooter = kExtensionPositionInchesHighCubeShooter
                * kExtensionRevolutionsPerInch;
        public static double kExtensionPositionCubeShooter = kExtensionPositionInchesCubeShooter
                * kExtensionRevolutionsPerInch;

        public static double kExtensionSafeExtensionMax = kExtensionPositionIntakeOut
                + kExtensionPositionTolerance;

        public static double kExtensionConeRetractDistance = 5;

    }

    public static final class AltitudeConstants {
        public static int kAltitudeMotorPort = 5;
        public static double kAltitudeStopSpeed = 0;
        public static double kAltitudeRaiseSpeed = 0.2;
        public static double kAltitudeLowerSpeed = 0.2;
        public static double kAltitudeRampRate = 1;
        public static double kAltitudePositionTolerance = 20;
        public static double kAutoAltitudePositionTolerance = 2;

        public static final double kMaxAltitudeSpeedMetersPerSecond = .05;
        public static final double kMaxLimitedAltitudeSpeedMetersPerSecond = .01;

        // Estimates, fix this once we get exact measurements
        public static final double kAltitudeTotalDegrees = 72.4;
        public static final double kAltitudeTotalRevolutions = 117.9;

        // Convert angle of travel to encoder rotations, where encoder reading of .1 is
        // 0 degrees and reading of 5.5 is 90 degrees
        public static final double kAltitudeRevolutionsPerDegree = -(kAltitudeTotalRevolutions)
                / kAltitudeTotalDegrees;

        // angle conversions for 2.14.2023 altitude readings
        // Altitude at Top Position
        public static double kAltitudeTravelPositionDegrees = 1;
        // Altitude at DropOff Position
        public static double kAltitudeDropOffPositionDegrees = 27;
        // Altitude when delivering high cone that it lowers to after extending fully
        public static double kAltitudeHighDropOffPositionDegrees = 36.8;
        // Altitude at position that it releases the cone
        public static double kAltitudeMidDropOffPositionDegrees = 32.0;
        // Altitude at position that it releases the High cone
        public static double kAltitudeHighDropOffFinalPositionDegrees = 41.4;
        // Altitude at position that it releases the Mid cone
        public static double kAltitudeMidDropOffFinalPositionDegrees = 42.8;
        // Altitude at high cube shoot Altitude
        public static double kAltitudeHighCubeShootPositionDegrees = 11.8;
        // Altitude at Bottom Position
        public static double kAltitudeIntakePositionDegrees = 71.1;
        // Altitude at Intake Position for Autos
        public static double kAltitudeAutoIntakePositionDegrees = 68;
        // Altitude at Shelf Position
        public static double kAltitudeDoubleSubstationPositionDegrees = 36.2;
        public static double kAltitudeSingleSubstationPositionDegrees = 9.;

        // encoder readings of altitude as of 2.14.2023
        // Altitude at Top Position
        public static double kAltitudeTravelPosition = kAltitudeTravelPositionDegrees
                * kAltitudeRevolutionsPerDegree;
        // Altitude at DropOff Position
        public static double kAltitudeDropOffPosition = kAltitudeDropOffPositionDegrees
                * kAltitudeRevolutionsPerDegree;
        // Altitude when delivering high cone that it lowers to after extending fully
        public static double kAltitudeHighDropOffPosition = kAltitudeHighDropOffPositionDegrees
                * kAltitudeRevolutionsPerDegree;
        // Altitude at position that it releases the cone
        public static double kAltitudeMidDropOffPosition = kAltitudeMidDropOffPositionDegrees
                * kAltitudeRevolutionsPerDegree;
        // Altitude at position that it releases the High cone
        public static double kAltitudeHighDropOffFinalPosition = kAltitudeHighDropOffFinalPositionDegrees
                * kAltitudeRevolutionsPerDegree;
        // Altitude at position that it releases the Mid cone
        public static double kAltitudeMidDropOffFinalPosition = kAltitudeMidDropOffFinalPositionDegrees
                * kAltitudeRevolutionsPerDegree;
        // Altitude at high cube shoot Altitude
        public static double kAltitudeHighCubeShootPosition = kAltitudeHighCubeShootPositionDegrees
                * kAltitudeRevolutionsPerDegree;
        // Altitude at Bottom Position
        public static double kAltitudeIntakePosition = kAltitudeIntakePositionDegrees
                * kAltitudeRevolutionsPerDegree;
        // Altitude at Intake Position for Autos
        public static double kAltitudeAutoIntakePosition = kAltitudeAutoIntakePositionDegrees
                * kAltitudeRevolutionsPerDegree;

        // Altitude at Shelf Position
        public static double kAltitudeDoubleSubstationPosition = kAltitudeDoubleSubstationPositionDegrees
                * kAltitudeRevolutionsPerDegree;
        public static double kAltitudeSingleSubstationPosition = kAltitudeSingleSubstationPositionDegrees
                * kAltitudeRevolutionsPerDegree;

        // Altitude Error Tolerance
        public static double kAltitudeError = 1;

        // Range for safe Travel altitude > -0.7
        public static double kAltitudeSafeMin = -18;

        public static double kAltitudeSafeExtensionMin = kAltitudeDropOffPosition - kAltitudePositionTolerance;

    }

    public static final class LimelightConstants {
        public static final int APRILTAG_PIPELINE = 0;

        public static final double VISION_POS_TOLLERANCE = 0.5;

        public static final double ALIGNED_GRID_APRIL_X = -12.0;
        public static final double ALIGNED_GRID_APRIL_Y = -3.0;
        public static final double ALIGNED_GRID_APRIL_AREA = 3.7;

        public static final double ALIGNED_SUBSTATION_APRIL_X = -18.3;
        public static final double ALIGNED_SUBSTATION_APRIL_Y = -16.3;
        public static final double ALIGNED_SUBSTATION_APRIL_AREA = 6.0;

        public static final double ALIGNED_LEFT_CONE_X = -18.3;
        public static final double ALIGNED_LEFT_CONE_Y = -16.3;
        public static final double ALIGNED_LEFT_CONE_AREA = 6.0;

        public static final double ALIGNED_RIGHT_CONE_X = -18.3;
        public static final double ALIGNED_RIGHT_CONE_Y = -16.3;
        public static final double ALIGNED_RIGHT_CONE_AREA = 6.0;

        public static final double SETPOINT_DIS_FROM_MID_CONE = 24;
        public static final double SETPOINT_DIS_FROM_TOP_CONE = 40;

        public static final double SETPOINT_DIS_FROM_GRID_APRIL = 14.062222;
        public static final double SETPOINT_DIS_FROM_SUBSTATION_APRIL = 5;
        // height of vision tape center in inches
        public static final double HEIGHT_CONE_NODE_TAP = 24.125;
        public static final double HEIGHT_GRID_APRIL = 18.25;
        public static final double HEIGHT_SUBSTATION_APRIL = 27.375;

        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0.0, 0.0, 0.0),
                new Rotation3d(0.0, 0.0, 0.0));
    }

    public static final class DriveConstants {
        public static final double kSafeSpeedLimit = 0.40; // % of max speed
        public static final double kSafeRotLimit = .2;

        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        // Note - we had changed this to .5 * Pi to slow down rotation
        public static final double kMaxAngularSpeed = 1 * Math.PI; // radians per second

        // Note - we had changed this to 2.4 direction, 3.6 magnitude, and 4.0
        // rotational
        public static final double kDirectionSlewRate = 2.4; // radians per second
        public static final double kMagnitudeSlewRate = 2.6; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 4.0; // percent per second (1 = 100%)

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

        // Gyro Constants
        public static final int kGyroDeviceNumber = 15;
        public static final boolean kGyroReversed = false;

        // Turn factor constants
        public static final double kTrackWidthInches = 20.5; // Known
        public static final double kTurnFactor = kTrackWidthInches * Math.PI / 360; // Known
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
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps
                * kWheelCircumferenceMeters)
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

        public static final int kDrivingMotorCurrentLimit = 30; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double kDriveDeadband = 0.05;
    }

    public static final class AutoConstants {
        // 4.5 / 4
        public static final double kMaxSpeedMetersPerSecond = 4.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = 4 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 4 * Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 4;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
        public static final double kTurnToleranceDeg = 2;
        public static final double kTurnRateToleranceDegPerS = 8;
        public static final double kAutoGyroTolerance = 2;

        public static final double kDriveOnStationMaxSpeed = 0.15;
        public static final double kDriveToBalanceFactor = 0.7;
        public static final double kFinalBalanceSpeed = kDriveOnStationMaxSpeed * (kDriveToBalanceFactor / 2);
    }

    public static final class OldAutoConstants {
        public static final double kOldAutoMaxSpeedMetersPerSecond = .4;
        public static final double kOldAutoMaxAccelerationMetersPerSecondSquared = 0.5;
        public static final double kOldAutoMaxAngularSpeedRadiansPerSecond = 0.25 * Math.PI;
        public static final double kOldAutoMaxAngularSpeedRadiansPerSecondSquared = 0.25 * Math.PI;

        public static final double kOldAutoPXController = 1;
        public static final double kOldAutoPYController = 1;
        public static final double kOldAutoPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kOldAutoMaxAngularSpeedRadiansPerSecond, kOldAutoMaxAngularSpeedRadiansPerSecondSquared);

        // Constants for use in Auto commands
        public static final double kTravelForwards = 1;
        public static final double kTravelBackwards = -1;

        public static final double kDriveToStationSpeed = 0.4;
        public static final double kDriveOnStationMaxSpeed = 0.15;
        public static final double kDriveToBalanceFactor = 0.7;
        public static final double kFinalBalanceSpeed = kDriveOnStationMaxSpeed * (kDriveToBalanceFactor / 2);
        public static final double kAutoGyroTolerance = 2;

    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }
}