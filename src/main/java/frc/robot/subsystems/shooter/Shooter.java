// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io; // The IO Implementation; transfers data to/from hardware
  private final ShooterIOInputsAutoLogged inputs =
      new ShooterIOInputsAutoLogged(); // The IO Layer; Transfers Data; Fills Class w/ Numbers every
  // 20ms

  // Two Separate SysId Routines
  private final SysIdRoutine flywheelSysId;
  private final SysIdRoutine hoodSysId;

  // Shooter Constructor
  public Shooter(ShooterIO io) {
    this.io = io;

    // Flywheel SysId Routine
    flywheelSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Default config
                (state) -> Logger.recordOutput("FlywheelSysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setFlywheelVoltage(voltage.in(Units.Volts)),
                null, // No log consumer, data recorded by AdvantageKit
                this));

    // Hood SysId Routine
    hoodSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Default config
                (state) -> Logger.recordOutput("HoodSysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setHoodVoltage(voltage.in(Units.Volts)),
                null, // No log consumer, data recorded by AdvantageKit
                this));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
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

  // Hood SysId Commands
  public Command hoodQuasistaticForward() {
    return hoodSysId.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command hoodQuasistaticReverse() {
    return hoodSysId.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command hoodDynamicForward() {
    return hoodSysId.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command hoodDynamicReverse() {
    return hoodSysId.dynamic(SysIdRoutine.Direction.kReverse);
  }
}

// need to do main logic next meeting
