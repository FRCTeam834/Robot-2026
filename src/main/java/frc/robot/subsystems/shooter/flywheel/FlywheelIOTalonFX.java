// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX flywheelMotor;
  private double flywheelVolts;

  public FlywheelIOTalonFX(int canId, String canBus) {
    flywheelMotor = new TalonFX(9, canBus);
    this.flywheelVolts = flywheelMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.flywheelConnected = flywheelMotor.isConnected();
    inputs.flywheelVelocityRadsPerSec =
        Units.rotationsToRadians(flywheelMotor.getVelocity().getValueAsDouble());
    inputs.flywheelAppliedVoltage = flywheelMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    this.flywheelVolts = MathUtil.clamp(volts, -12.0, 12.0);
    flywheelMotor.setVoltage(this.flywheelVolts);
  }

  //public void setFlywheelVelocity(double velocityRadsPerSec, VoltageUnit feedforwardVolts){}

  public void updateFlywheelPID(Slot0Configs config) {
    flywheelMotor.getConfigurator().apply(config);
  }
}
