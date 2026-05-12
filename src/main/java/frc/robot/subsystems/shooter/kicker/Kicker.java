// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.kicker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.kicker.KickerConstants.KickerState;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();
  private KickerState kickerState;

  public Kicker(KickerIO io) {
    this.io = io;
    kickerState = KickerState.STOP;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
  }

  // Kicker Setter Methods
  public void setKickerVoltage(double volts) {
    io.setKickerVoltage(volts);
  }

  public void setDesiredKickerState(KickerState kickerState) {
    this.kickerState = kickerState;
    io.setKickerVoltage(kickerState.voltage);
  }

  // Kicker Getter Methods
  public double getKickerRPM() {
    return inputs.kickerRPM;
  }

  public double getKickerCurrent() {
    return inputs.kickerCurrent;
  }

  public double getKickerVoltage() {
    return inputs.kickerAppliedVoltage;
  }

  public void stopMotor() {
    io.stopMotor();
  }
}
