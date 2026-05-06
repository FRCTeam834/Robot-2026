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
  private final TalonFX lowerFlywheelMotor;
  private final TalonFX upperFlywheelMotor;
  public SimpleMotorFeedforward flywheelFeedforward;
  private final VelocityVoltage velocitySetPoint;
  private final TalonFXConfiguration upperFlywheelConfig;
  private final TalonFXConfiguration lowerFlywheelConfig;


  public FlywheelIOTalonFX() {
    lowerFlywheelMotor = new TalonFX(9, "canBus"); 
    upperFlywheelMotor = new TalonFX(10, "canBus");
    
    flywheelFeedforward = new SimpleMotorFeedforward(0, 0);
    velocitySetPoint = new VelocityVoltage(0.0).withSlot(0);
    lowerFlywheelConfig = new TalonFXConfiguration();
    upperFlywheelConfig = new TalonFXConfiguration();

    upperFlywheelConfig.Slot0.kP = 0.1;
    upperFlywheelConfig.Slot0.kI = 0.0;
    upperFlywheelConfig.Slot0.kD = 0.0;
    upperFlywheelConfig.Slot0.kS = 0.01;
    upperFlywheelConfig.Slot0.kV = 0.01;
    upperFlywheelConfig.Slot0.kA = 0.0;
    upperFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    upperFlywheelConfig.CurrentLimits.SupplyCurrentLimit = 40.0;

    upperFlywheelConfig.Voltage.PeakForwardVoltage = 12.0;
    upperFlywheelConfig.Voltage.PeakReverseVoltage = -12.0;

    lowerFlywheelConfig.Slot0.kP = 0.1;
    lowerFlywheelConfig.Slot0.kI = 0.0;
    lowerFlywheelConfig.Slot0.kD = 0.0;
    lowerFlywheelConfig.Slot0.kS = 0.01;
    lowerFlywheelConfig.Slot0.kV = 0.01;
    lowerFlywheelConfig.Slot0.kA = 0.0;
    lowerFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    lowerFlywheelConfig.CurrentLimits.SupplyCurrentLimit = 40.0;

    lowerFlywheelConfig.Voltage.PeakForwardVoltage = 12.0;
    lowerFlywheelConfig.Voltage.PeakReverseVoltage = -12.0;

    upperFlywheelMotor.getConfigurator().apply(upperFlywheelConfig);
    
    lowerFlywheelMotor.getConfigurator().apply(lowerFlywheelConfig);

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
    lowerFlywheelMotor.setVoltage(MathUtil.clamp(targetVolts, -12.0, 12.0));
  }

  @Override
  public void setFlywheelVelocity(double targetRPM) {
    var velocity = RPM.of(targetRPM);
    double rps = Units.rotationsPerMinuteToRadiansPerSecond(targetRPM);
    double ffVolts = flywheelFeedforward.calculate(rps);
    upperFlywheelMotor.setControl(velocitySetPoint.withVelocity(velocity).withFeedForward(ffVolts));
    lowerFlywheelMotor.setControl(velocitySetPoint.withVelocity(velocity).withFeedForward(ffVolts));
  }

  @Override
  public void updateFlywheelPID(Slot0Configs config) {
    upperFlywheelMotor.getConfigurator().apply(config);
    lowerFlywheelMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateFlywheelFeedforward(double kS, double kV) {
    this.flywheelFeedforward = new SimpleMotorFeedforward(kS, kV);
  }

  @Override
  public void stopMotor() {
    upperFlywheelMotor.stopMotor();
    lowerFlywheelMotor.stopMotor();
  }
}
