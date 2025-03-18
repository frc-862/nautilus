package frc.robot.util;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Manages a number value published to the root table of NT. */
public class NetworkNumber {
  private final DoubleEntry entry;
  private double defaultValue = 0.0;
  private double value;

  /**
   * Creates a new LoggedNetworkNumber, for handling a number input sent via
   * NetworkTables.
   *
   * @param key The key for the number, published to the root table of NT or
   *            "/DashboardInputs/{key}" when logged.
   */
  public NetworkNumber(String key) {
    this.entry = NetworkTableInstance.getDefault().getDoubleTopic(key).getEntry(0.0);
    this.value = defaultValue;
  }

  /**
   * Creates a new LoggedNetworkNumber, for handling a number input sent via
   * NetworkTables.
   *
   * @param key          The key for the number, published to the root table of NT
   *                     or "/DashboardInputs/{key}" when logged.
   * @param defaultValue The default value if no value in NT is found.
   */
  public NetworkNumber(String key, double defaultValue) {
    this(key);
    setDefault(defaultValue);
    this.value = defaultValue;
  }

  /** Updates the default value, which is used if no value in NT is found. */
  public void setDefault(double defaultValue) {
    this.defaultValue = defaultValue;
    entry.set(entry.get(defaultValue));
  }

  /**
   * Publishes a new value. Note that the value will not be returned by
   * {@link #get()} until the next cycle.
   */
  public void set(double value) {
    entry.set(value);
  }

  /** Returns the current value. */
  public double get() {
    return value;
  }

  public void periodic() {
    value = entry.get(defaultValue);
  }
}
