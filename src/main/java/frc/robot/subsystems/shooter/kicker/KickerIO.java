package frc.robot.subsystems.shooter.kicker;

import com.revrobotics.spark.config.SparkFlexConfig;
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    public boolean kickerConnected = false;
    double kickerRPM = 0.0;
    double kickerAppliedVoltage = 0.0;
  }

  public default void updateInputs(KickerIOInputs inputs) {}

  public default void setKickerVelocity(double targetRPM) {}

  public default void setKickerVoltage(double volts) {}

  public default void updateKickerPID(SparkFlexConfig kickerConfig) {}

  public default void updateKickerFeedforward(double kS, double kV) {}
}
