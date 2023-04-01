// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  NetworkTable tableLeft = NetworkTableInstance.getDefault().getTable("limelight-left");
  NetworkTable tableRight = NetworkTableInstance.getDefault().getTable("limelight-right");

  public static HttpCamera m_limelightLeft;
  public static HttpCamera m_limelightRight;

  private boolean m_visionMode;

  public LimelightSubsystem() {
    m_limelightLeft = new HttpCamera("LeftLL", "http://limelight-left:5809/stream.mjpg");
    m_limelightLeft.setResolution(320, 240);
    m_limelightLeft.setFPS(90);
    m_limelightRight = new HttpCamera("RightLL", "http://limelight-right:5809/stream.mjpg");
    m_limelightRight.setResolution(320, 240);
    m_limelightRight.setFPS(90);

    CameraServer.addCamera(m_limelightLeft);
    CameraServer.addCamera(m_limelightRight);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // read values periodically
    // SmartDashboard.putData(SendableCameraWrapper.wrap(m_limelightLeft));
    // SmartDashboard.putData(SendableCameraWrapper.wrap(m_limelightRight));
    if (Constants.TuningModeConstants.kLimelightTuning) {
      SmartDashboard.putNumber("X offset", getXRight());
      SmartDashboard.putNumber("Y offset", getYRight());
      SmartDashboard.putNumber("Target Area", getAreaRight());

      SmartDashboard.putNumber("Bot X", getBotPoseBlue().getX());
      SmartDashboard.putNumber("Bot y", getBotPoseBlue().getY());
      SmartDashboard.putNumber("Bot Rot", getBotPoseBlue().getZ());
    }

  }

  public double getXLeft() {
    return tableLeft.getEntry("tx").getDouble(0.0);
  }

  public double getYLeft() {
    return tableLeft.getEntry("ty").getDouble(0.0);
  }

  public double getAreaLeft() {
    return tableLeft.getEntry("ta").getDouble(0.0);
  }

  public boolean hasTargetLeft() {
    return tableLeft.getEntry("tv").getDouble(0.0) == 1;
  }

  public int getIDLeft() {
    return (int) tableLeft.getEntry("tid").getDouble(0.0);
  }

  public double getLatPipLeft() {
    return tableLeft.getEntry("tl").getDouble(0.0) / 1000.0;
  }

  public double getLatCapLeft() {
    return tableLeft.getEntry("cl").getDouble(0.0) / 1000.0;
  }

  public Pose3d getBotPose() {
    double[] pose = tableLeft.getEntry("botpose").getDoubleArray(new double[6]);
    return new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), new Rotation3d(pose[3], pose[4], pose[5]));
  }

  public Pose3d getBotPoseBlue() {
    double[] pose = tableLeft.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    return new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), new Rotation3d(pose[3], pose[4], pose[5]));
  }

  public Pose3d getBotPoseRed() {
    double[] pose = tableLeft.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    return new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), new Rotation3d(pose[3], pose[4], pose[5]));
  }

  public Transform3d getTransform() {
    return new Transform3d(new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)), getBotPose());
  }

  public double getLastEntryTimeStamp() {
    return Timer.getFPGATimestamp() - getLatCapLeft() - getLatPipLeft();
  }

  /**
   * @param piplineNumber driver = 0, aprilTags = 1, retroreflective = 2
   */
  public void setPipelineLeft(int pipelineNumber) {
    Number numObj = (Number) pipelineNumber;
    tableLeft.getEntry("pipeline").setNumber(numObj);
  }

  public double getXRight() {
    return tableRight.getEntry("tx").getDouble(0.0);
  }

  public double getYRight() {
    return tableRight.getEntry("ty").getDouble(0.0);
  }

  public double getAreaRight() {
    return tableRight.getEntry("ta").getDouble(0.0);
  }

  public boolean hasTargetRight() {
    return tableRight.getEntry("tv").getDouble(0.0) == 1;
  }

  public int getIDRight() {
    return (int) tableRight.getEntry("tid").getDouble(0.0);
  }

  /**
   * @param piplineNumber 0 = april tags
   */
  public void setPipelineRight(int pipelineNumber) {
    Number numObj = (Number) pipelineNumber;
    tableRight.getEntry("pipeline").setNumber(numObj);
  }

  public boolean inVisionMode() {
    return m_visionMode;
  }

  public void setVisionModeOn() {
    m_visionMode = true;
  }

  public void setVisionModeOff() {
    m_visionMode = false;
  }
}
