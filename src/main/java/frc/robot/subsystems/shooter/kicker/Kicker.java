// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.kicker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.Constants;
import com.revrobotics.spark.config.SparkFlexConfig;


public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  public static final LoggedTunableNumber kicker_kP = new LoggedTunableNumber("Kicker/kicker_kP");
  public static final LoggedTunableNumber kicker_kS = new LoggedTunableNumber("Kicker/kicker_kS");
  public static final LoggedTunableNumber kicker_kV = new LoggedTunableNumber("Kicker/kicker_kV");


  public Kicker(KickerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);

    if (Constants.tuningMode && kicker_kP.hasChanged(hashCode())
        || kicker_kS.hasChanged(hashCode())
        || kicker_kV.hasChanged(hashCode())) {
      var kickerConfig = new SparkFlexConfig();
      kickerConfig.closedLoop.p(kicker_kP.get());
      io.updateKickerPID(kickerConfig);
      io.updateKickerFeedforward(kicker_kS.get(), kicker_kV.get());
    }
  }
  }
