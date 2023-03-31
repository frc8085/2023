// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AlignWithGridApril extends CommandBase {
  /** Creates a new AlignWithNodeApril. */
  LimelightSubsystem m_limelight;
  DriveSubsystem m_drive;
  PIDController m_pidControllerY, m_pidControllerX;
  boolean m_end;
  double xTrans = 0.0;
  double yTrans = 0.0;
  double count = 0;
  private PIDController m_thetaController;

  public AlignWithGridApril(LimelightSubsystem limelight, DriveSubsystem drive) {
    m_drive = drive;
    m_limelight = limelight;
    addRequirements(m_drive, m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidControllerY = new PIDController(LimelightConstants.kP, LimelightConstants.kI,
        LimelightConstants.kD);
    m_pidControllerY.setIntegratorRange(0.0, LimelightConstants.kZ);
    m_pidControllerY.setTolerance(LimelightConstants.VISION_POS_TOLLERANCE);

    m_pidControllerX = new PIDController(LimelightConstants.kP, LimelightConstants.kI,
        LimelightConstants.kD);
    m_pidControllerX.setIntegratorRange(0.0, LimelightConstants.kZ);
    m_pidControllerX.setTolerance(LimelightConstants.VISION_POS_TOLLERANCE);

    m_thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);

    m_thetaController.enableContinuousInput(-180, 180);

    m_end = false;

    m_limelight.setVisionModeOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_limelight.hasTargetRight()) {
      m_end = true;
      System.out.println("No target");
    }

    m_thetaController.setSetpoint(180.0);

    double rotationVal = m_thetaController.calculate((MathUtil.inputModulus(
        m_drive.getYaw().getDegrees(), -180, 180)), 180.0);
    rotationVal = MathUtil.clamp(rotationVal, -AutoConstants.kMaxAngularSpeedRadiansPerSecond * 0.4,
        AutoConstants.kMaxAngularSpeedRadiansPerSecond * 0.4);

    m_pidControllerX.setSetpoint(LimelightConstants.ALIGNED_GRID_APRIL_X);
    xTrans = m_pidControllerX.calculate(m_limelight.getXRight());
    xTrans = MathUtil.clamp(xTrans, -0.5, 0.5);

    m_pidControllerY.setSetpoint(LimelightConstants.ALIGNED_GRID_APRIL_AREA);
    yTrans = m_pidControllerY.calculate(m_limelight.getAreaRight());
    yTrans = MathUtil.clamp(yTrans, -0.5, 0.5);

    m_drive.driveWithPose(new Translation2d(-yTrans, -xTrans), rotationVal, true, true);

    if (Constants.TuningModeConstants.kLimelightTuning) {
      SmartDashboard.putNumber("X Error", m_pidControllerX.getPositionError());
      SmartDashboard.putNumber("Y Error", m_pidControllerY.getPositionError());
    }

    if (m_pidControllerX.atSetpoint() && m_pidControllerY.atSetpoint() && count > 50) {
      m_end = true;
    } else {
      m_end = false;
    }

    count++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
    m_limelight.setVisionModeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_end;
  }
}
