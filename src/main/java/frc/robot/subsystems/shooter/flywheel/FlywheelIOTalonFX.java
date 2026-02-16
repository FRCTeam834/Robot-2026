// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX flywheelMotor;
  public SimpleMotorFeedforward flywheelFeedforward;
  private final VelocityVoltage velocitySetPoint;

  public FlywheelIOTalonFX(int canId, String canBus) {
    flywheelMotor = new TalonFX(9, canBus); // Change Later
    flywheelFeedforward = new SimpleMotorFeedforward(0, 0);
    velocitySetPoint = new VelocityVoltage(0.0).withSlot(0);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.flywheelConnected = flywheelMotor.isConnected();
    inputs.flywheelVelocityRPM = flywheelMotor.getVelocity().getValueAsDouble() * 60;
    inputs.flywheelAppliedVoltage = flywheelMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setFlywheelVoltage(double targetVolts) {
    flywheelMotor.setVoltage(MathUtil.clamp(targetVolts, -12.0, 12.0));
  }

  @Override
  public void setFlywheelVelocity(double targetRPM) {
    double targetRotationsPerSec = targetRPM / 60;
    double ffVolts = flywheelFeedforward.calculate(targetRotationsPerSec);

    flywheelMotor.setControl(
        velocitySetPoint.withVelocity(targetRotationsPerSec).withFeedForward(ffVolts));
  }

  @Override
  public void updateFlywheelPID(Slot0Configs config) {
    flywheelMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateFlywheelFeedforward(double kS, double kV) {
    this.flywheelFeedforward = new SimpleMotorFeedforward(kS, kV);
  }
}
