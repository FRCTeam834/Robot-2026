// Copyright (c) FIRST and other
// WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public static final LoggedTunableNumber roller_kP = new LoggedTunableNumber("Intake/roller_kP");
  public static final LoggedTunableNumber roller_kS = new LoggedTunableNumber("Intake/roller_kS");
  public static final LoggedTunableNumber roller_kV = new LoggedTunableNumber("Intake/roller_kV");

  public static final LoggedTunableNumber pivot_kP = new LoggedTunableNumber("Intake/pivot_kP");
  public static final LoggedTunableNumber pivot_kS = new LoggedTunableNumber("Intake/pivot_kS");
  public static final LoggedTunableNumber pivot_kG = new LoggedTunableNumber("Intake/pivot_kG");
  public static final LoggedTunableNumber pivot_kV = new LoggedTunableNumber("Intake/pivot_kV");

  final SysIdRoutine rollerSysId;

  public Intake(IntakeIO io) {
    this.io = io;

    rollerSysId =
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
    Logger.processInputs("Pivot", inputs);

    // Pivot
    if (Constants.tuningMode && pivot_kP.hasChanged(hashCode())
        || pivot_kS.hasChanged(hashCode())
        || pivot_kV.hasChanged(hashCode())
        || pivot_kG.hasChanged(hashCode())) {
      var pivotConfig = new SparkMaxConfig();
      pivotConfig.closedLoop.p(pivot_kP.get());
      io.updatePivotPID(pivotConfig);
      io.updatePivotFeedforward(pivot_kS.get(), pivot_kG.get(), pivot_kV.get());
    }

    // Roller
    if (Constants.tuningMode && roller_kP.hasChanged(hashCode())
        || roller_kS.hasChanged(hashCode())
        || roller_kV.hasChanged(hashCode())) {
      var rollerConfig = new SparkFlexConfig();
      rollerConfig.closedLoop.p(roller_kP.get());
      io.updateRollerPID(rollerConfig);
      io.updateRollerFeedforward(roller_kV.get(), roller_kS.get());
    }
  }
}
