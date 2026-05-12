package frc.robot.subsystems.shooter.kicker;

public class KickerConstants {
  public static enum KickerState {
    FAST(8.0),
    SLOW(3.0),
    REVERSE(-3.0),
    STOP(0.0);

    public final double voltage;

    private KickerState(double voltage) {
      this.voltage = voltage;
    }
  };
}
