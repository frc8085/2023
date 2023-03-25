// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import frc.robot.Constants.AltitudeConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ExtensionConstants;
import frc.robot.Constants.TuningModeConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private boolean TUNING_MODE = TuningModeConstants.kDriveTuning;
    private Altitude m_altitude;
    private Extension m_extension;
    public SwerveDriveOdometry m_odometry;
    public MAXSwerveModule[] m_SwerveMods;

    public WPI_Pigeon2 m_gyro;
    public Field2d m_field;

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private double[] pitchYawRollVelocitiesDegreesPerSecond = new double[3];

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem(Altitude Altitude, Extension Extension) {
        m_gyro = new WPI_Pigeon2(DriveConstants.kGyroDeviceNumber, "drive");
        m_gyro.configFactoryDefault();
        zeroGyro();
        m_field = new Field2d();

        m_altitude = Altitude;
        m_extension = Extension;

        m_SwerveMods = new MAXSwerveModule[] {
                m_frontLeft,
                m_frontRight,
                m_rearLeft,
                m_rearRight
        };

        // Odometry class for tracking robot pose
        m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getYaw(),
                new SwerveModulePosition[] { m_frontLeft.getPosition(), m_frontRight.getPosition(),
                        m_rearLeft.getPosition(), m_rearRight.getPosition() });

    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                getYaw(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });

    // calculate velocities for pitch, yaw, roll from the gyro
    // Returned in degrees per second
    m_gyro.getRawGyro(pitchYawRollVelocitiesDegreesPerSecond);

    log();

        m_field.setRobotPose(m_odometry.getPoseMeters());

        SmartDashboard.putData("Field", m_field);
        if (TUNING_MODE) {
            SmartDashboard.putNumber("gyro yaw", getYaw().getDegrees());

            // SmartDashboard.putNumber("Front Left Position", m_frontLeft.getPosition());
            // SmartDashboard.putString("Front Left State", m_frontLeft.getState());
        }
    }

    public void testDrive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        m_odometry.getPoseMeters().getRotation())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                DriveConstants.kMaxSpeedMetersPerSecond);

        SmartDashboard.putNumber("X Translation", translation.getX());
        SmartDashboard.putNumber("Y Translation", translation.getY());
        SmartDashboard.putNumber("Rotation Value", rotation);

    }

    public void stopDrive() {
        testDrive(new Translation2d(0, 0), 0, false, true);
    }

  private void log() {
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putNumber("X Pose", getPose().getX());
    SmartDashboard.putNumber("Y Pose", getPose().getY());
    SmartDashboard.putNumber("Yaw", getYaw());

    SmartDashboard.putNumberArray("XYZ_DPS", pitchYawRollVelocitiesDegreesPerSecond);
  }

    // Returns the currently-estimated pitch of the robot.
    public double getPitch() {
        return m_gyro.getPitch();
    }

    public double getRoll() {
        return m_gyro.getRoll();
    }

    public void setYaw(double yaw) {
        m_gyro.setYaw(yaw);
    }

    public Rotation2d getYaw() {
        return (false) ? Rotation2d.fromDegrees(360 - m_gyro.getYaw())
                : Rotation2d.fromDegrees(m_gyro.getYaw());
    }

    public void zeroGyro() {
        m_gyro.setYaw(0);
    }

    /**
     * Returns the currently-estimated roll velocity of the robot.
     *
     * @return The roll velocity (degrees per second)
     */
    public double getRollVelocity() {
      return pitchYawRollVelocitiesDegreesPerSecond[2];
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                getYaw(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);
    }

    public void reset() {
        zeroGyro();
        Timer.delay(0.04);
        resetOdometry(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0)));
    }



  /**
   * Returns the currently-estimated pitch velocity of the robot.
   *
   * @return The pitch velocity (degrees per second)
   */
  public double getPitchVelocity() {
    return pitchYawRollVelocitiesDegreesPerSecond[0];
    // return 0;
  }

  /**
   * Returns the currently-estimated pitch of the robot.
   *
   * @return The rotation.
   */
  public Rotation2d getRotation() {
    return m_gyro.getRotation2d();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to stop the robot
   */
  public void stop() {
    drive(0, 0, 0, 0, false, false);
  }

  public void turn(double speed) {
    drive(0, 0, 0, speed, true, false);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */

  // TODO: confirm that rotation speed is reduced when altitude and extension are
  // not in travel and that there is no slew then
  public void drive(double speed, double xSpeed, double ySpeed, double rot, boolean fieldRelative,
      boolean rateLimit) {
    double speedCommanded = isWithinSafeDrivingLimits() ? speed : DriveConstants.kSafeSpeedLimit * speed;
    double xSpeedCommanded;
    double ySpeedCommanded;
    double rotSpeed = isWithinSafeDrivingLimits() ? rot : DriveConstants.kSafeRotLimit * rot;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(speedCommanded * ySpeed, speedCommanded * xSpeed);
      double inputTranslationMag = Math
          .sqrt(Math.pow(speedCommanded * xSpeed, 2) + Math.pow(speedCommanded * ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality
                                              // checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                m_odometry.getPoseMeters().getRotation())
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void lock() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.setYaw(0);
    }

    // Gyro Stuff
    public double getAngle() {
        return -m_gyro.getYaw();
    }

    public Rotation2d getRotation2d() {
        // Rotation2d and Pigeon are both ccw+
        return Rotation2d.fromDegrees(m_gyro.getYaw());
    }

    // public double getRate() {
    // getRawGyro(_xyz_dps);
    // return -_xyz_dps[2];
    // }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, not wrapped
     */
    public double getHeading() {
        return getRotation2d().getDegrees();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeadingWrappedDegrees() {
        return MathUtil.inputModulus(getRotation2d().getDegrees(), -180, 180);
    }

    // /**
    // * Returns the turn rate of the robot.
    // *
    // * @return The turn rate of the robot, in degrees per second
    // */
    // public double getTurnRate() {
    // return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    // }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public boolean isWithinSafeDrivingLimits() {
    boolean altitudeInSafeLimit = m_altitude.getCurrentAltitude() > AltitudeConstants.kAltitudeSafeMin;
    boolean extenstionInSafeLimit = m_extension
        .getCurrentExtensionPosition() < ExtensionConstants.kExtensionSafeMax;

    // If in Auto, it is safe to drive faster
    // But if in Teleop, consider us within safe driving limits only if the altitude
    // and extension are within safe limits
    return RobotState.isAutonomous() || (altitudeInSafeLimit && extenstionInSafeLimit);
  }
}