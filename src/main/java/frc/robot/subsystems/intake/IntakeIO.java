package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // Rollers
    public boolean rollerConnected = false;
    public double rollerRPM = 0.0;
    public double rollerAppliedVoltage = 0.0;

    // Pivot
    public boolean pivotConnected = false;
    public double pivotPositionRads = 0.0;
    public double pivotAppliedVoltage = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRollerVoltage(double volts) {}

  public default void setRollerVelocity(double targetRPM, double ffVolts) {} // need to fix

  public default void setPivotVoltage(double volts) {}

  public default void setPivotPosition(double pivotPositionRads) {}

  public default void updatePivotPID(SparkMaxConfig pivotMaxConfig, double kS, double kG) {}
}
