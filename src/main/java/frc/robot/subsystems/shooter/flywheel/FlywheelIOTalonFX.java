// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;


public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX lowerFlywheelMotor;
  private final TalonFX upperFlywheelMotor;
  private final VelocityVoltage velocityRequest;
  private final TalonFXConfiguration flywheelConfig;


  public FlywheelIOTalonFX() {
    lowerFlywheelMotor = new TalonFX(9); 
    upperFlywheelMotor = new TalonFX(10);
    velocityRequest = new VelocityVoltage(0.0).withSlot(0);
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

    upperFlywheelMotor.getConfigurator().apply(flywheelConfig);
    lowerFlywheelMotor.getConfigurator().apply(flywheelConfig);

    upperFlywheelMotor.setControl(new Follower(lowerFlywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.lowerFlywheelVelocityRPM = lowerFlywheelMotor.getVelocity().getValueAsDouble()*60; //mult by 60 to go from rot/sec to rot/min
    inputs.lowerFlywheelCurrent = lowerFlywheelMotor.getTorqueCurrent().getValueAsDouble();
    inputs.lowerFlywheelDutyCycle = lowerFlywheelMotor.getDutyCycle().getValueAsDouble();

    inputs.upperFlywheelVelocityRPM = upperFlywheelMotor.getVelocity().getValueAsDouble()*60; //mult by 60 to go from rot/sec to rot/min
    inputs.upperFlywheelCurrent = upperFlywheelMotor.getTorqueCurrent().getValueAsDouble();
    inputs.upperFlywheelDutyCycle = upperFlywheelMotor.getDutyCycle().getValueAsDouble();
  }

  @Override
  public void setFlywheelVoltage(double targetVolts) {
    upperFlywheelMotor.setVoltage(MathUtil.clamp(targetVolts, -12.0, 12.0));
  }

  @Override
  public void setFlywheelVelocity(double targetRPM) {
    upperFlywheelMotor.setControl(velocityRequest.withVelocity(targetRPM));
  }

  @Override
  public void updateFlywheelPID(Slot0Configs config) {
    upperFlywheelMotor.getConfigurator().apply(config);
    lowerFlywheelMotor.getConfigurator().apply(config);
  }

  @Override
  public void stopMotor() {
    upperFlywheelMotor.stopMotor();
  }
}
