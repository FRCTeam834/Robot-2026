// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX flywheelMotor;
  private final VelocityTorqueCurrentFOC velocitySetpoint;

  public FlywheelIOTalonFX() {
    flywheelMotor = new TalonFX(9);
    velocitySetpoint = new VelocityTorqueCurrentFOC(0.0).withSlot(0);

    var flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.withSlot0(FlywheelConstants.flywheelConfig);
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelMotor.getConfigurator().apply(flywheelConfig);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.flywheelConnected = flywheelMotor.isConnected();
    inputs.flywheelVelocityRPM = flywheelMotor.getVelocity().getValueAsDouble();
    inputs.flywheelCurrent = flywheelMotor.getTorqueCurrent().getValueAsDouble();
    inputs.flywheelDutyCycle = flywheelMotor.getDutyCycle().getValueAsDouble();
  }

  /*
   * -1 to 1
   */
  @Override
  public void setFlywheelDutyCycle(double dutyCycle) {
    flywheelMotor.setControl(new DutyCycleOut(dutyCycle));
  }

  @Override
  public void setFlywheelVelocity(double targetRPM) {
    flywheelMotor.setControl(velocitySetpoint.withVelocity(RPM.of(targetRPM)));
  }

  @Override
  public void updateClosedLoopConfig(Slot0Configs config) {
    flywheelMotor.getConfigurator().apply(config);
  }

  @Override
  public void stopMotor() {
    flywheelMotor.stopMotor();
  }
}
