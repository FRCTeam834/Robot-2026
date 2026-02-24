package frc.robot.subsystems.shooter.kicker;

import com.revrobotics.spark.config.SparkFlexConfig;
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    public boolean kickerConnected = false;
    public double kickerRPM = 0.0;
    public double kickerAppliedVoltage = 0.0;
  }

  public default void updateInputs(KickerIOInputs inputs) {}

  public default void setKickerVoltage(double targetVolts) {}

  public default void setKickerVelocity(double targetRPM) {}

  public default void updateKickerPID(SparkFlexConfig kickerConfig) {}

  public default void updateKickerFeedforward(double kS, double kV) {}

  public default void stopMotor() {}
}
