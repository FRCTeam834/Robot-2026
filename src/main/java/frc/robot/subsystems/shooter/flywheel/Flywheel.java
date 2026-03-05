// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.FlywheelState;
import frc.robot.util.LoggedTunableNumber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private DoubleSupplier hubDistance;

  public static final LoggedTunableNumber flywheel_kP =
      new LoggedTunableNumber("Flywheel/flywheel_kP");
  public static final LoggedTunableNumber flywheel_kS =
      new LoggedTunableNumber("Flywheel/flywheel_kS");
  public static final LoggedTunableNumber flywheel_kV =
      new LoggedTunableNumber("Flywheel/flywheel_kV");
  public static final LoggedTunableNumber manual_flywheel_setpoint =
      new LoggedTunableNumber("Flywheel/manual_rpm_setpoint");

  static{
    flywheel_kP.initDefault(0);
    flywheel_kS.initDefault(0);
    flywheel_kV.initDefault(0);
    manual_flywheel_setpoint.initDefault(0);
  }

  @AutoLogOutput(key = "SubsystemStates/FlywheelState") private FlywheelState flywheelState;

  public Flywheel(FlywheelIO io, DoubleSupplier hubDistance) {
    this.io = io;
    this.hubDistance = hubDistance;

    flywheelState = FlywheelState.STOPPED;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    if (Constants.TUNING_MODE
        && (flywheel_kP.hasChanged(hashCode())
            || flywheel_kS.hasChanged(hashCode())
            || flywheel_kV.hasChanged(hashCode()))) {
      var flywheelConfig = new Slot0Configs();
      flywheelConfig.kP = flywheel_kP.get();
      flywheelConfig.kS = flywheel_kS.get();
      flywheelConfig.kV = flywheel_kV.get();
      io.updateClosedLoopConfig(flywheelConfig);
    }

    switch(flywheelState) {
      case ACTIVE -> {
        double setpointVelocityRPM = ShotCalculator.flywheelRPMForDistance(hubDistance.getAsDouble());
        io.setFlywheelVelocity(setpointVelocityRPM);
      }
      case IDLE -> io.setFlywheelVelocity(FlywheelConstants.idleRPM);
      case MANUAL_RPM -> io.setFlywheelVelocity(manual_flywheel_setpoint.get());
      case STOPPED -> io.stopMotor();
    }
  }

  public double getFlywheelRPM() {
    return inputs.flywheelVelocityRPM;
  }

  public void setDesiredState(FlywheelState state) {
    flywheelState = state;
  }

  // Miscellaneous Methods
  public boolean isAtSetpointRPM(double targetRPM) {
    return MathUtil.isNear(targetRPM, getFlywheelRPM(), FlywheelConstants.toleranceRPM);
  }

  public FlywheelState getState() {
    return flywheelState;
  }

  public void stopMotor() {
    io.stopMotor();
  }
}
