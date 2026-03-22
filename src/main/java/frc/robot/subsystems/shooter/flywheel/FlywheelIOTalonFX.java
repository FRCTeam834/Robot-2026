// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX flywheelMotor1;
  private final TalonFX flywheelMotor2;

  private final VelocityTorqueCurrentFOC velocityTorqueRequest =
      new VelocityTorqueCurrentFOC(0.0).withSlot(0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public FlywheelIOTalonFX() {
    flywheelMotor1 = new TalonFX(30);
    flywheelMotor2 = new TalonFX(32);

    var flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.withSlot0(FlywheelConstants.flywheelConfig);

    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelConfig.CurrentLimits.StatorCurrentLimit = 50;
    flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    flywheelMotor1.getConfigurator().apply(flywheelConfig);
    flywheelMotor2.getConfigurator().apply(flywheelConfig);

    flywheelMotor2.setControl(
        new Follower(flywheelMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.ONE_flywheelConnected = flywheelMotor1.isConnected();
    inputs.ONE_flywheelVelocityRPM = flywheelMotor1.getVelocity().getValue().in(RPM);
    inputs.ONE_flywheelCurrent = flywheelMotor1.getTorqueCurrent().getValueAsDouble();
    inputs.ONE_flywheelDutyCycle = flywheelMotor1.getDutyCycle().getValueAsDouble();

    inputs.TWO_flywheelConnected = flywheelMotor2.isConnected();
    inputs.TWO_flywheelCurrent = flywheelMotor2.getTorqueCurrent().getValueAsDouble();
  }

  /*
   * -1 to 1
   */
  @Override
  public void setFlywheelVoltage(double volts) {
    flywheelMotor1.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setFlywheelVelocity(double targetRPM) {
    flywheelMotor1.setControl(velocityTorqueRequest.withVelocity(RPM.of(targetRPM)));
  }

  @Override
  public void updateClosedLoopConfig(Slot0Configs config) {
    flywheelMotor1.getConfigurator().apply(config);
  }

  @Override
  public void stopMotor() {
    flywheelMotor1.stopMotor();
  }
}
