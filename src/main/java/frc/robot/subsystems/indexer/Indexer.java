// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private LoggedTunableNumber indexer_kP = new LoggedTunableNumber("Indexer/indexer_kP");
  private LoggedTunableNumber indexer_kS = new LoggedTunableNumber("Indexer/indexer_kS");
  private LoggedTunableNumber indexer_kV = new LoggedTunableNumber("Indexer/indexer_kV");

  final SysIdRoutine indexerSysId;

  public Indexer(IndexerIO io) {
    this.io = io;

    indexerSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("IndexerRollerSysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (Voltage voltage) -> io.setIndexerVoltage(voltage.in(Units.Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    if (Constants.tuningMode && indexer_kP.hasChanged(hashCode())
        || indexer_kS.hasChanged(hashCode())
        || indexer_kV.hasChanged(hashCode())) {
      var indexerConfig = new SparkFlexConfig();
      indexerConfig.closedLoop.p(indexer_kP.get());
      io.updateIndexerPID(indexerConfig);
      io.updateIndexerFeedforward(indexer_kV.get(), indexer_kS.get());
    }
  }
}
