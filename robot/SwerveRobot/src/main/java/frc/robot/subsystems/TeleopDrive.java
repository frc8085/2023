// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import frc.robot.Constants.AltitudeConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ExtensionConstants;
import frc.robot.Constants.TuningModeConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TeleopDrive extends CommandBase {
    private DriveSubsystem m_drive;
    private double m_speed;
    private DoubleSupplier xSup;
    private DoubleSupplier ySup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier m_halfSpeed;
    private BooleanSupplier m_quarterSpeed;
    private BooleanSupplier m_90, m_180, m_270, m_0;

    private double rotationVal, xVal, yVal;
    double m_angle = 0d;
    private PIDController m_thetaController;
    private SendableChooser<Double> m_speedChooser;

    private boolean TUNING_MODE = TuningModeConstants.kDriveTuning;
    private Altitude m_altitude;
    private Extension m_extension;

    // PID coefficients
    static double kPAngleSnap = 0.02;
    static double kIAngleSnap = 0;
    static double kDAngleSnap = 0;
    // static double kIzAngleSnap = 0;
    static double kFFAngleSnap = 0;
    static double kMaxOutputAngleSnap = .5;
    static double kMinOutputAngleSnap = -.5;

    public static final double Snap_Tolerance = 2.0;

    /**
     * 
     * @param s_Swerve
     * @param xSup
     * @param ySup
     * @param rotationSup
     * @param robotCentricSup
     * @param halfSpeed
     * @param quarterSpeed
     */
    public TeleopDrive(DriveSubsystem drive, double speed, DoubleSupplier xSup, DoubleSupplier ySup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup,
            BooleanSupplier halfSpeed, BooleanSupplier quarterSpeed,
            BooleanSupplier zero, BooleanSupplier ninety, BooleanSupplier oneEighty, BooleanSupplier twoSeventy) {
        m_drive = drive;
        m_speed = speed;
        this.ySup = ySup;
        this.xSup = xSup;
        this.rotationSup = rotationSup;
        m_halfSpeed = halfSpeed;
        m_quarterSpeed = quarterSpeed;
        m_0 = zero;
        m_180 = oneEighty;
        m_90 = ninety;
        m_270 = twoSeventy;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_thetaController = new PIDController(
                kPAngleSnap,
                kIAngleSnap,
                kDAngleSnap);

        m_thetaController.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {

        /* Get Values, Deadband */
        boolean rotateWithButton = m_0.getAsBoolean() || m_90.getAsBoolean() || m_180.getAsBoolean()
                || m_270.getAsBoolean();
        xVal = MathUtil.applyDeadband(xSup.getAsDouble() * m_speedChooser.getSelected(),
                0.02);
        yVal = MathUtil.applyDeadband(ySup.getAsDouble() * m_speedChooser.getSelected(),
                0.02);
        // SmartDashboard.putBoolean("rotate with button", rotateWithButton);

        if (rotateWithButton) {
            if (m_0.getAsBoolean()) {
                m_thetaController.setSetpoint(0.0);
            } else if (m_90.getAsBoolean()) {
                m_thetaController.setSetpoint(-90.0);
            } else if (m_180.getAsBoolean()) {
                m_thetaController.setSetpoint(180.0);
            } else if (m_270.getAsBoolean()) {
                m_thetaController.setSetpoint(90.0);
            }
            rotationVal = m_thetaController.calculate(
                    (MathUtil.inputModulus(m_drive.getPose().getRotation().getDegrees(), -180, 180)),
                    m_thetaController.getSetpoint());
            rotationVal = MathUtil.clamp(rotationVal, -DriveConstants.kMaxAngularSpeed * 0.075,
                    DriveConstants.kMaxAngularSpeed * 0.075);
            // SmartDashboard.putNumber("RotationVal", rotationVal);
            // SmartDashboard.putNumber("Theta Controller setpoint",
            // m_thetaController.getSetpoint());
        } else if (!rotateWithButton) {
            rotationVal = (MathUtil.applyDeadband(rotationSup.getAsDouble() * m_speedChooser.getSelected(),
                    0.02)) * 0.75;
        }

        if (m_quarterSpeed.getAsBoolean()) {
            xVal = xVal * 0.25;
            yVal = yVal * 0.25;
            if (!rotateWithButton) {
                rotationVal = rotationVal * 0.25;
            }
        } else if (m_halfSpeed.getAsBoolean()) {
            xVal = xVal * 0.5;
            yVal = yVal * 0.5;
            if (!rotateWithButton) {
                rotationVal = rotationVal * 0.5;
            }
        } else if (m_altitude.AltitudeIsInTravelPosition() != true) {
            xVal = xVal * 0.45;
            yVal = yVal * 0.45;
            if (!rotateWithButton) {
                rotationVal = rotationVal * 0.45;
            }
        } else {
            xVal = xVal * 1.0;
            yVal = yVal * 1.0;
            if (!rotateWithButton) {
                rotationVal = rotationVal * 1.0;
            }
        }

        m_drive.testDrive(
                new Translation2d(xVal, yVal).times(DriveConstants.kMaxSpeedMetersPerSecond),
                rotationVal * DriveConstants.kMaxAngularSpeed * 0.9,
                true,
                false);
    }
}
