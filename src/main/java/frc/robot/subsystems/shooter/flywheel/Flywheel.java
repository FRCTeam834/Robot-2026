// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
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
    SysIdRoutine flywheelSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("FlywheelSysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (Voltage voltage) -> io.setFlywheelVoltage(voltage.in(Units.Volts)), null, this));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    if (Constants.tuningMode
        && flywheelkP.hasChanged(hashCode())
        && flywheelkS.hasChanged(hashCode())
        && flywheelkV.hasChanged(hashCode())) {
      var flywheelConfig = new Slot0Configs();
      flywheelConfig.kP = flywheelkP.get();
      flywheelConfig.kS = flywheelkS.get();
      flywheelConfig.kV = flywheelkV.get();
      io.updateFlywheelPID(flywheelConfig);
    }
  }

  // Flywheel SysId Commands
  public Command flywheelQuasistaticForward(SysIdRoutine flywheelSysId) {
    return flywheelSysId.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command flywheelQuasistaticReverse(SysIdRoutine flywheelSysId) {
    return flywheelSysId.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command sysIdFlywheelDynamicForward(SysIdRoutine flywheelSysId) {
    return flywheelSysId.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdFlywheelDynamicReverse(SysIdRoutine flywheelSysId) {
    return flywheelSysId.dynamic(SysIdRoutine.Direction.kReverse);
  }
}
