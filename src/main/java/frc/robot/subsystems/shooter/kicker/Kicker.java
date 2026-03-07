// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.kicker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.kicker.KickerConstants.KickerState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  @AutoLogOutput(key = "SubsystemStates/KickerState")
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

  public void setKickerState(KickerState state) {
    io.setKickerVoltage(kickerState.voltage);
    kickerState = state;
  }

  @AutoLogOutput
  public double getKickerRPM() {
    return inputs.kickerRPM;
  }
}
