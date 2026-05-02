package frc.robot.subsystems.indexer;

public class IndexerConstants {
  public static enum IndexerState {
    FAST(8.0),
    SLOW(4.0),
    REVERSE(-3.0),
    STOP(0);

    public final double voltage;

    private IndexerState(double voltage) {
      this.voltage = voltage;
    }
  };
}
