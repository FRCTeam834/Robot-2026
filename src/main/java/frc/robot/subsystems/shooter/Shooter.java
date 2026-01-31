// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io; // The IO Implementation; transfers data to/from hardware
  private final ShooterIOInputsAutoLogged inputs =
      new ShooterIOInputsAutoLogged(); // The IO Layer; Transfers Data; Fills Class w/ Numbers every
  // 20ms

  private final SysIdRoutine flywheelSysId;

  // Interpolating Tables
  public static InterpolatingDoubleTreeMap shotAngleTable = new InterpolatingDoubleTreeMap();
  public static InterpolatingDoubleTreeMap shotSpeedTable = new InterpolatingDoubleTreeMap();

  // PID Shooter
  private static final LoggedTunableNumber flywheelkP =
      new LoggedTunableNumber("Shooter/flywheelkP");
  private static final LoggedTunableNumber flywheelkS =
      new LoggedTunableNumber("Shooter/flywheelkS");
  private static final LoggedTunableNumber flywheelkV =
      new LoggedTunableNumber("Shotter/flywheelkV");

  // PID Hood
  private static final LoggedTunableNumber hoodkP = new LoggedTunableNumber("Shooter/hoodkP");
  private static final LoggedTunableNumber hoodkS = new LoggedTunableNumber("Shooter/hoodkS");
  private static final LoggedTunableNumber hoodkV = new LoggedTunableNumber("Shooter/hoodkV");

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

  // What was the thought on this piece of code? We will have to write code for the underlined
  // methods soon

  /*

    public boolean atSetpoint(double distance) {
      double setpointRPM = shotSpeedTable.get(distance);
      double setpointAngle = shotAngleTable.get(distance);
      return Math.abs(setpointRPM - getCurrentTopRollerSpeed())
          && Math.abs(setpointRPM - getCurrentButtonRollerSpeed())
          && Math.abs(setpointAngle - getCurrentPivotAngle()) <= toleranceAngle;
    }


  */

  // currently working on PID (1/29)
}

// need to do main logic next meeting
// interpolation - check 2024 robot
// class InterpolationDoubleTreeMap
