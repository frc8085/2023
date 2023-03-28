package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TuningModeConstants;
import frc.utils.LimelightConfiguration.Advanced_Crosshair;
import frc.utils.LimelightConfiguration.Advanced_Target;
import frc.utils.LimelightConfiguration.CamMode;
import frc.utils.LimelightConfiguration.LedMode;
import frc.utils.LimelightConfiguration.Snapshot;
import frc.utils.LimelightConfiguration.StreamType;

/**
 * Lime Light Class was started by Corey Applegate of Team 3244
 * Granite City Gearheads. We Hope you Enjoy the Lime Light
 * Camera.
 */
public class Limelight extends SubsystemBase {
  private boolean TUNING_MODE = TuningModeConstants.kLimelightTuning;
  private NetworkTable m_table;
  private String m_tableName;
  private Boolean isConnected = false;
  private double _hearBeatPeriod = 0.1;
  private NetworkTableEntry targetVisible;
  private NetworkTableEntry distanceToTarget;
  private NetworkTableEntry rotationToTarget;
  private NetworkTableEntry targetSetpoint;
  private double setpointTarget = 0;

  MedianFilter distanceFilter = new MedianFilter(5);
  MedianFilter rotationFilter = new MedianFilter(5);

  class PeriodicRunnable implements java.lang.Runnable {
    public void run() {
      resetPilelineLatency();
      setpointTarget = getSetpointToTarget();
      log();

      try {
        Thread.sleep(50);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
      if (getPipelineLatency() == 0.0) {
        isConnected = false;
      } else {
        isConnected = true;
      }
    }
  }

  Notifier _hearBeat = new Notifier(new PeriodicRunnable());

  /** The log method puts interesting information to the SmartDashboard. */
  public void log() {
    // Things to show only in tuninig mode
    if (TUNING_MODE) {
      SmartDashboard.putBoolean("LL Sees target", getIsTargetFound());
      SmartDashboard.putNumber("LL Distance", getDistanceToTarget());
      SmartDashboard.putNumber("LL Rotation", getdegRotationToTarget());
      SmartDashboard.putNumber("LL Setpoint", getSetpointToTarget());
    }
  }

  /**
   * Using the Default Lime Light NT table
   */
  public Limelight() {
    m_tableName = "limelight";
    m_table = NetworkTableInstance.getDefault().getTable(m_tableName);
    _hearBeat.startPeriodic(_hearBeatPeriod);

  }

  /**
   * If you changed the name of your Lime Light tell Me the New Name
   */
  public Limelight(String tableName) {
    m_tableName = tableName;
    m_table = NetworkTableInstance.getDefault().getTable(m_tableName);
    _hearBeat.startPeriodic(_hearBeatPeriod);
  }

  /**
   * Send an instance of the NetworkTabe
   */
  public Limelight(NetworkTable table) {
    m_table = table;
    _hearBeat.startPeriodic(_hearBeatPeriod);
  }

  // This is a test
  public boolean isConnected() {
    return isConnected;
  }

  /**
   * Use the area of the bounding rectagle to calculate our distance to the target
   * (from the camera)
   * polynomial function determined empricially based on the following data
   * gathered
   * --- | -----
   * in. | ta
   * 
   */
  public double getDistanceToTargetWithArea() {
    double targetArea = getTargetArea();
    double distanceToTargetInches = 38.3225 * Math.pow(targetArea, 2) - 169.91 * targetArea + 179.71;

    // Return the distance to the target
    return targetArea > 0 ? distanceToTargetInches - 20 : 0;
  }

  public double getDistanceToTarget() {
    double tY = getdegVerticalToTarget();
    double distanceToTargetInches = -3.5429 * tY + 94.438;

    // Return the distance to the target
    return distanceToTargetInches - 9;
  }

  public double getLiveSetpointTarget() {
    return setpointTarget;
  }

  public double getSetpointToTarget() {
    double distance = getDistanceToTarget();
    // Empirically derived formula
    double autoSetpoint = 0.1607 * Math.pow(distance, 2) - 28.274 * distance + 4991.1;

    // When we're too close the setpoint formla is unreliable. Fix the speed
    // instead.
    double setpoint = distance < 100 ? 3600 : autoSetpoint;

    // Return the distance to the target
    return setpoint;
  }

  /**
   * tv Whether the limelight has any valid targets (0 or 1)
   * 
   * @return
   */
  public boolean getIsTargetFound() {
    NetworkTableEntry tv = m_table.getEntry("tv");
    double v = tv.getDouble(0);
    if (v == 0.0f) {
      return false;
    } else {
      return true;
    }
  }

  /**
   * tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
   * 
   * @return
   */
  public double getdegRotationToTarget() {
    NetworkTableEntry tx = m_table.getEntry("tx");
    double x = tx.getDouble(0.0);
    return x;
  }

  /**
   * ty Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
   * 
   * @return
   */
  public double getdegVerticalToTarget() {
    NetworkTableEntry ty = m_table.getEntry("ty");
    double y = ty.getDouble(0.0);
    return y;
  }

  /**
   * ta Target Area (0% of image to 100% of image)
   * 
   * @return
   */
  public double getTargetArea() {
    NetworkTableEntry ta = m_table.getEntry("ta");
    double a = ta.getDouble(0.0);
    return a;
  }

  /**
   * ts Skew or rotation (-90 degrees to 0 degrees)
   * 
   * @return
   */
  public double getSkew_Rotation() {
    NetworkTableEntry ts = m_table.getEntry("ts");
    double s = ts.getDouble(0.0);
    return s;
  }

  /**
   * tl The pipeline’s latency contribution (ms) Add at least 11ms for image
   * capture latency.
   * 
   * @return
   */
  public double getPipelineLatency() {
    NetworkTableEntry tl = m_table.getEntry("tl");
    double l = tl.getDouble(0.0);
    return l;
  }

  private void resetPilelineLatency() {
    m_table.getEntry("tl").setValue(0.0);
  }
  // Setters

  /**
   * LedMode Sets limelight’s LED state
   * 
   * kon
   * koff
   * kblink
   * 
   * @param ledMode
   */
  public void setLEDMode(LedMode ledMode) {
    m_table.getEntry("ledMode").setValue(ledMode.getValue());
  }

  /**
   * Returns current LED mode of the Lime Light
   * 
   * @return LedMode
   */
  public LedMode getLEDMode() {
    NetworkTableEntry ledMode = m_table.getEntry("ledMode");
    double led = ledMode.getDouble(0.0);
    LedMode mode = LedMode.getByValue(led);
    return mode;
  }

  /**
   * camMode Sets limelight’s operation mode
   * 
   * kvision
   * kdriver (Increases exposure, disables vision processing)
   * 
   * @param camMode
   */

  public void setCamMode(CamMode camMode) {
    m_table.getEntry("camMode").setValue(camMode.getValue());
  }

  /**
   * Returns current Cam mode of the Lime Light
   * 
   * @return CamMode
   */
  public CamMode getCamMode() {
    NetworkTableEntry camMode = m_table.getEntry("camMode");
    double cam = camMode.getDouble(0.0);
    CamMode mode = CamMode.getByValue(cam);
    return mode;
  }

  /**
   * pipeline Sets limelight’s current pipeline
   * 
   * 0 . 9 Select pipeline 0.9
   * 
   * @param pipeline
   */
  public void setPipeline(Integer pipeline) {
    if (pipeline < 0) {
      pipeline = 0;
      throw new IllegalArgumentException("Pipeline can not be less than zero");
    } else if (pipeline > 9) {
      pipeline = 9;
      throw new IllegalArgumentException("Pipeline can not be greater than nine");
    }
    m_table.getEntry("pipeline").setValue(pipeline);
  }

  /**
   * Returns current Pipeline of the Lime Light
   * 
   * @return Pipeline
   */
  public double getPipeline() {
    NetworkTableEntry pipeline = m_table.getEntry("pipeline");
    double pipe = pipeline.getDouble(0.0);
    return pipe;
  }

  /**
   * Returns current Pipeling of the Lime Light
   * 
   * @return Pipelinge
   */
  public Integer getPipelineInt() {
    NetworkTableEntry pipeline = m_table.getEntry("pipeline");
    Integer pipe = (int) pipeline.getDouble(0.0);
    return pipe;
  }

  /**
   * stream Sets limelight’s streaming mode
   * 
   * kStandard - Side-by-side streams if a webcam is attached to Limelight
   * kPiPMain - The secondary camera stream is placed in the lower-right corner of
   * the primary camera stream
   * kPiPSecondary - The primary camera stream is placed in the lower-right corner
   * of the secondary camera stream
   * 
   * @param stream
   */
  public void setStream(StreamType stream) {
    m_table.getEntry("stream").setValue(stream.getValue());
  }

  public StreamType getStream() {
    NetworkTableEntry stream = m_table.getEntry("stream");
    double st = stream.getDouble(0.0);
    StreamType mode = StreamType.getByValue(st);
    return mode;
  }

  /**
   * snapshot Allows users to take snapshots during a match
   * 
   * kon - Stop taking snapshots
   * koff - Take two snapshots per second
   * 
   * @param snapshot
   */
  public void setSnapshot(Snapshot snapshot) {
    m_table.getEntry("snapshot").setValue(snapshot.getValue());
  }

  public Snapshot getSnapshot() {
    NetworkTableEntry snapshot = m_table.getEntry("snapshot");
    double snshot = snapshot.getDouble(0.0);
    Snapshot mode = Snapshot.getByValue(snshot);
    return mode;
  }

  // *************** Advanced Usage with Raw Contours *********************

  /**
   * Limelight posts three raw contours to NetworkTables that are not influenced
   * by your grouping mode.
   * That is, they are filtered with your pipeline parameters, but never grouped.
   * X and Y are returned
   * in normalized screen space (-1 to 1) rather than degrees. *
   */

  public double getAdvanced_RotationToTarget(Advanced_Target raw) {
    NetworkTableEntry txRaw = m_table.getEntry("tx" + Integer.toString(raw.getValue()));
    double x = txRaw.getDouble(0.0);
    return x;
  }

  public double getAdvanced_degVerticalToTarget(Advanced_Target raw) {
    NetworkTableEntry tyRaw = m_table.getEntry("ty" + Integer.toString(raw.getValue()));
    double y = tyRaw.getDouble(0.0);
    return y;
  }

  public double getAdvanced_TargetArea(Advanced_Target raw) {
    NetworkTableEntry taRaw = m_table.getEntry("ta" + Integer.toString(raw.getValue()));
    double a = taRaw.getDouble(0.0);
    return a;
  }

  public double getAdvanced_Skew_Rotation(Advanced_Target raw) {
    NetworkTableEntry tsRaw = m_table.getEntry("ts" + Integer.toString(raw.getValue()));
    double s = tsRaw.getDouble(0.0);
    return s;
  }

  // Raw Crosshairs:
  // If you are using raw targeting data, you can still utilize your calibrated
  // crosshairs:

  public double[] getAdvanced_RawCrosshair(Advanced_Crosshair raw) {
    double[] crosshars = new double[2];
    crosshars[0] = getAdvanced_RawCrosshair_X(raw);
    crosshars[1] = getAdvanced_RawCrosshair_Y(raw);
    return crosshars;
  }

  public double getAdvanced_RawCrosshair_X(Advanced_Crosshair raw) {
    NetworkTableEntry cxRaw = m_table.getEntry("cx" + Integer.toString(raw.getValue()));
    double x = cxRaw.getDouble(0.0);
    return x;
  }

  public double getAdvanced_RawCrosshair_Y(Advanced_Crosshair raw) {
    NetworkTableEntry cyRaw = m_table.getEntry("cy" + Integer.toString(raw.getValue()));
    double y = cyRaw.getDouble(0.0);
    return y;
  }
}