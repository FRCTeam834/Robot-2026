package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean indexerConnected = false;
    public double indexerAppliedVoltage = 0.0;
  }
  
  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setIndexerVoltage(double targetVolts) {}

}
