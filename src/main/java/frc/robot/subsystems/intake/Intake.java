// Copyright (c) FIRST and other
// WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Voltage;


public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public static final LoggedTunableNumber pivot_kP = new LoggedTunableNumber("Pivot/pivot_kP");
  public static final LoggedTunableNumber pivot_kS = new LoggedTunableNumber("Pivot/pivot_kS");
  public static final LoggedTunableNumber pivot_kG = new LoggedTunableNumber("Pivot/pivot_kG");

  final SysIdRoutine intakeSysId;

  public Intake(IntakeIO io) {
    this.io = io;

    intakeSysId = 
      new SysIdRoutine(
        new SysIdRoutine.Config(
          null,
          null,
          null,
          (state) -> Logger.recordOutput("RollerSysIdTestState", state.toString())),
        new SysIdRoutine.Mechanism(
          (Voltage voltage) -> io.setRollerVoltage(voltage.in(Units.Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Roller", inputs);

    if (Constants.tuningMode && pivot_kP.hasChanged(hashCode())
        || pivot_kS.hasChanged(hashCode())
        || pivot_kG.hasChanged(hashCode())) {
      var pivotConfig = new SparkMaxConfig();
      pivotConfig.closedLoop.p(pivot_kP.get());
      io.updatePivotPID(pivotConfig, pivot_kS.get(), pivot_kG.get());
    }
  }
}
