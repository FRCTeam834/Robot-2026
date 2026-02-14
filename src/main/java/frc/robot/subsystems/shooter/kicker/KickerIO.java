package frc.robot.subsystems.shooter.kicker;

import com.ctre.phoenix6.configs.Slot0Configs;
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    double kickerRPM = 0.0;
    boolean kickeratSetpoint = false;
    double kickerAppliedVoltage = 0.0;
  }

  public default void updateInputs(KickerIOInputs inputs) {}

  public default void setKickerVelocity(double targetRPM, double feedforwardVolts) {}

  public default void setKickerVelocity(double volts) {}

  public default void updateFlywheelPID(Slot0Configs configs) {}
}
