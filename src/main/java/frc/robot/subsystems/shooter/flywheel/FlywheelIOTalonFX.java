// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX flywheelMotor;
  public SimpleMotorFeedforward flywheelFeedforward;
  private final VelocityVoltage velocitySetPoint;
  private final TalonFXConfiguration flywheelConfig;

  public FlywheelIOTalonFX() {
    flywheelMotor = new TalonFX(9, "canBus"); // Change Later
    flywheelFeedforward = new SimpleMotorFeedforward(0, 0);
    velocitySetPoint = new VelocityVoltage(0.0).withSlot(0);
    flywheelConfig = new TalonFXConfiguration();
    
    flywheelConfig.Slot0.kP = 0.1;
    flywheelConfig.Slot0.kI = 0.0;
    flywheelConfig.Slot0.kD = 0.0;
    flywheelConfig.Slot0.kS = 0.01;
    flywheelConfig.Slot0.kV = 0.01;
    flywheelConfig.Slot0.kA = 0.0;    
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelConfig.CurrentLimits.SupplyCurrentLimit = 40.0;

    flywheelConfig.Voltage.PeakForwardVoltage = 12.0;
    flywheelConfig.Voltage.PeakReverseVoltage = -12.0;
    
    flywheelMotor.getConfigurator().apply(flywheelConfig);
   
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
    var velocity = RPM.of(targetRPM);
    double rps = Units.rotationsPerMinuteToRadiansPerSecond(targetRPM);
    double ffVolts = flywheelFeedforward.calculate(rps);
    flywheelMotor.setControl(
        velocitySetPoint.withVelocity(velocity).withFeedForward(ffVolts));
  }

  @Override
  public void updateFlywheelPID(Slot0Configs config) {
    flywheelMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateFlywheelFeedforward(double kS, double kV) {
    this.flywheelFeedforward = new SimpleMotorFeedforward(kS, kV);
  }

  @Override
  public void stopMotor() {
    flywheelMotor.stopMotor();
  }
}
