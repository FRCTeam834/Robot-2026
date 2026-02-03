// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private static final LoggedTunableNumber flywheelkP =
      new LoggedTunableNumber("Flywheel/flywheelkP");
  private static final LoggedTunableNumber flywheelkS =
      new LoggedTunableNumber("Flywheel/flywheelkS");
  private static final LoggedTunableNumber flywheelkV =
      new LoggedTunableNumber("Flywheel/flywheelkV");

  public Flywheel(FlywheelIO io) {
    this.io = io;

    // Flywheel SysId Routine
    flywheelSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("FlywheelSysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setFlywheelVoltage(voltage.in(Units.Volts)), null, this));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  // Flywheel SysId Commands
  public Command flywheelQuasistaticForward() {
    return flywheelSysId.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command flywheelQuasistaticReverse() {
    return flywheelSysId.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command sysIdFlywheelDynamicForward() {
    return flywheelSysId.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdFlywheelDynamicReverse() {
    return flywheelSysId.dynamic(SysIdRoutine.Direction.kReverse);
  }

  public Flywheel() {}
}
