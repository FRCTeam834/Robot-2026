package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.ClosedLoopConfig;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // Rollers
    public double rollerRPM = 0.0;
    public double rollerAppliedVoltage = 0.0;
    public double rollerCurrent = 0.0;

    // Pivot
    public double pivotPositionRads = 0.0;
    public double pivotAppliedVoltage = 0.0;
    public double pivotRPM = 0.0;
    public double pivotCurrent = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRollerVoltage(double targetVolts) {}

  public default void setPivotVoltage(double volts) {}

  public default void setPivotAngle(double angle) {}

  public default void updateClosedLoopConfig(ClosedLoopConfig config) {}

  public default void stopPivot() {}
  public default void stopRollers() {}
}
