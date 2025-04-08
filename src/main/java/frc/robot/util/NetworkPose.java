package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Manages a number value published to the root table of NT. */
public class NetworkPose {
  private final DoubleArrayEntry entry;
  private double[] defaultValue = {0.0, 0.0, 0.0}; // Default pose array for NT
  private double[] value = {0.0, 0.0, 0.0}; // Default pose array for NT

  /**
   * Creates a new LoggedNetworkNumber, for handling a number input sent via
   * NetworkTables.
   *
   * @param key The key for the number, published to the root table of NT or
   *            "/DashboardInputs/{key}" when logged.
   */
  public NetworkPose(String key) {
    this.value = defaultValue;
    this.entry = NetworkTableInstance.getDefault().getDoubleArrayTopic(key).getEntry(value);
  }

  /**
   * Creates a new LoggedNetworkNumber, for handling a number input sent via
   * NetworkTables.
   *
   * @param key          The key for the number, published to the root table of NT
   *                     or "/DashboardInputs/{key}" when logged.
   * @param defaultValue The default value if no value in NT is found.
   */
  public NetworkPose(String key, Pose2d defaultValue) {
    this(key);
    setDefault(defaultValue);
    this.value = this.defaultValue;
  }

  /** Updates the default value, which is used if no value in NT is found.
   * @param defaultValue The default value if no value in NT is found.
   */
 public void setDefault(Pose2d defaultValue) {
    this.defaultValue = new double[] {defaultValue.getX(), defaultValue.getY(), defaultValue.getRotation().getRadians()};
    entry.set(entry.get(this.defaultValue));
  }

  /**
   * Publishes a new value. Note that the value will not be returned by
   * {@link #get()} until the next cycle.
   *
   * @param value The new value to publish.
   */
  public void set(Pose2d value) {
    double[] poseArray = {value.getX(), value.getY(), value.getRotation().getRadians()};
    entry.set(poseArray);
  }

  /** Returns the current value.
   * @return The current value, or the default value if no value is found in NT.
   */
  public Pose2d get() {
    return new Pose2d(entry.get(defaultValue)[0], entry.get(defaultValue)[1], 
                new Rotation2d(entry.get(defaultValue)[2]));
  }

  public void periodic() {
    value = entry.get(defaultValue);
  }
}
