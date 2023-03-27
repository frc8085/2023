/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.utils;

import java.util.HashMap;
import java.util.Map;

/**
 * A utility class to set configuration options for the LimeLight
 */
public class LimelightConfiguration {

  public enum LedMode {
    kpipeLine(0), // 0 use the LED Mode set in the current pipeline
    kforceOff(1), // 1 force off
    kforceBlink(2), // 2 force blink
    kforceOn(3); // 3 force on

    private static final Map<Double, LedMode> LedModes = new HashMap<Double, LedMode>();

    static {
      for (LedMode LedMode : values()) {
        LedModes.put(LedMode.getValue(), LedMode);
      }
    }

    private double mode;

    private LedMode(double mode) {
      this.mode = mode;
    }

    public double getValue() {
      return mode;
    }

    public static LedMode getByValue(double mode) {
      return LedModes.get(mode);
    }

    public String toString() {
      return name();
    }

  }

  public enum CamMode {
    kvision(0),
    kdriver(1);

    private static final Map<Double, CamMode> CamModes = new HashMap<Double, CamMode>();

    static {
      for (CamMode CamMode : values()) {
        CamModes.put(CamMode.getValue(), CamMode);
      }
    }

    private double mode;

    private CamMode(double mode) {
      this.mode = mode;
    }

    public double getValue() {
      return mode;
    }

    public static CamMode getByValue(double mode) {
      return CamModes.get(mode);
    }

    public String toString() {
      return name();
    }
  }

  public enum StreamType {
    kStandard(0),
    kPiPMain(1),
    kPiPSecondary(2);

    private static final Map<Double, StreamType> StreamTypes = new HashMap<Double, StreamType>();

    static {
      for (StreamType StreamType : values()) {
        StreamTypes.put(StreamType.getValue(), StreamType);
      }
    }

    private double mode;

    private StreamType(double mode) {
      this.mode = mode;
    }

    public double getValue() {
      return mode;
    }

    public static StreamType getByValue(double mode) {
      return StreamTypes.get(mode);
    }

    public String toString() {
      return name();
    }

  }

  public enum Snapshot {

    kon(1), koff(0);

    private static final Map<Double, Snapshot> SnapshotModes = new HashMap<Double, Snapshot>();

    static {
      for (Snapshot Snapshot : values()) {
        SnapshotModes.put(Snapshot.getValue(), Snapshot);
      }
    }

    private double mode;

    private Snapshot(double mode) {
      this.mode = mode;
    }

    public double getValue() {
      return mode;
    }

    public static Snapshot getByValue(double mode) {
      return SnapshotModes.get(mode);
    }

    public String toString() {
      return name();
    }

  }

  public enum Advanced_Target {

    kone(0), ktwo(1), kthree(2);

    private static final Map<Integer, Advanced_Target> AdvanedTargets = new HashMap<Integer, Advanced_Target>();

    static {
      for (Advanced_Target Advanced_Target : values()) {
        AdvanedTargets.put(Advanced_Target.getValue(), Advanced_Target);
      }
    }

    private Integer target;

    private Advanced_Target(Integer target) {
      this.target = target;
    }

    public Integer getValue() {
      return target;
    }

    public static Advanced_Target getByValue(Integer target) {
      return AdvanedTargets.get(target);
    }

    public String toString() {
      return name();
    }

  }

  public enum Advanced_Crosshair {

    kone(0), ktwo(1);

    private static final Map<Integer, Advanced_Crosshair> AdvancedCrosshairs = new HashMap<Integer, Advanced_Crosshair>();

    static {
      for (Advanced_Crosshair Advanced_Crosshair : values()) {
        AdvancedCrosshairs.put(Advanced_Crosshair.getValue(), Advanced_Crosshair);
      }
    }

    private Integer crosshair;

    private Advanced_Crosshair(Integer crosshair) {
      this.crosshair = crosshair;
    }

    public Integer getValue() {
      return crosshair;
    }

    public static Advanced_Crosshair getByValue(Integer crosshair) {
      return AdvancedCrosshairs.get(crosshair);
    }

    public String toString() {
      return name();
    }

  }
}