// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import com.ctre.phoenix6.configs.Slot0Configs;




/** Add your docs here. */
public class HoodIOSparkMax implements HoodIO {
    private SparkMax hoodMotor;
    private double hoodVolts = hoodMotor.getBusVoltage();
    SparkAbsoluteEncoder absEncoder = hoodMotor.getAbsoluteEncoder();
    private double velocity = absEncoder.getVelocity();
    private double positionRads = absEncoder.getPosition();

    public HoodIOSparkMax (int canId, String canBus) {
        hoodMotor = new SparkMax(canId, null);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodConnected = true;
        inputs.hoodPositionRads = positionRads;
        inputs.hoodVelocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocity);
        inputs.hoodAppliedVoltage = hoodVolts;



    }

    public void setHoodVoltage(double volts) {
        this.hoodVolts = MathUtil.clamp(volts, -12.0, 12.0);
        hoodMotor.setVoltage(this.hoodVolts);
    }

    @Override
    public void updateHoodPID(Slot0Configs config) {
        hoodMotor.getConfigurator().apply(config); // FIX PID IMPLEMENTATION
    }

}
