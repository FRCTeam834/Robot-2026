// package frc.robot.subsystems.shooter;

// import edu.wpi.first.units.VoltageUnit;
// import org.littletonrobotics.junction.AutoLog;

// // The TalonFX objects moved there.

// // The TalonFXConfiguration (PID gains, current limits) moved there.

// // The VelocityVoltage objects moved there.

// public interface ShooterIO {
//   @AutoLog
//   public static class ShooterIOInputs {

//     // Flywheel
//     public boolean flywheelConnected = false;
//     public double flywheelVelocityRadsPerSec = 0.0;
//     public double flywheelAppliedVoltage = 0.0;
//     // public double flywheelTorqueCurrentAmps = 0.0;

//     // Hood
//     public boolean hoodConnected = false;
//     public double hoodPositionRads = 0.0;
//     public double hoodVelocityRadsPerSec = 0.0;
//     public double hoodAppliedVoltage = 0.0;
//     // public double hoodTorqueCurrentAmps = 0.0;
//   }

//   public default void updateInputs(ShooterIOInputs inputs) {}

//   public default void setFlywheelVelocity(
//       double velocityRadsPerSec, VoltageUnit feedforwardVolts) {}

//   public default void setHoodPosition(double positionRads, VoltageUnit feedforwardVolts) {}

//   public default void setFlywheelVoltage(double volts) {}

//   public default void setHoodVoltage(double volts) {}
// }
