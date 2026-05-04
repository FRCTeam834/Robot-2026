package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkFlexConfig;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // Rollers
    public double rollerAppliedVoltage = 0.0;
    public double rollerCurrent = 0.0;
    public double rollerRPM = 0.0;

    // Pivot
    public double pivotCurrent = 0.0;
    public double pivotAppliedVoltage = 0.0;
    public double pivotPositionRads = 0.0;
    public double pivotVelocity = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRollerVoltage(double targetVolts) {}

  public default void setPivotVoltage(double volts) {}

  public default void setPivotPosition(double targetPosition) {}

  public default void updatePivotClosedLoopConfig(SparkFlexConfig config) {}

  public default void stopRollerMotor() {}

  public default void stopPivotMotor() {}

  // public default void updatePivotFeedforward(double kS, double kG, double kV){}
}
