// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.Slot0Configs;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    // public boolean ONE_flywheelConnected = false;
    public double ONE_flywheelVelocityRPM;
    public double ONE_flywheelCurrent;
    public double ONE_flywheelDutyCycle;

    // public boolean TWO_flywheelConnected = false;
    public double TWO_flywheelCurrent;
    public double TWO_flywheelDutyCycle;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setFlywheelVelocity(double RPM) {}

  public default void setFlywheelVoltage(double volts) {}

  public default void updateClosedLoopConfig(Slot0Configs config) {}

  public default void stopMotor() {}
}
