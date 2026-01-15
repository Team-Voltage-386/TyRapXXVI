package frc.robot.util;

import java.util.OptionalDouble;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TuningUtil {

  private final LoggedNetworkNumber network;
  private double value;

  public TuningUtil(String key, double value) {
    network = new LoggedNetworkNumber(key, value);
    this.value = value;
  }

  public double getValue() {
    return network.get();
  }

  public OptionalDouble get() {
    double ntval = network.get();
    if (ntval != value) {
      value = ntval;
      return OptionalDouble.of(ntval);
    } else {
      return OptionalDouble.empty();
    }
  }
}
