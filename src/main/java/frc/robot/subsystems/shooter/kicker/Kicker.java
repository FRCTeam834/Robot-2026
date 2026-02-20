// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.kicker;

import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  public static final LoggedTunableNumber kicker_kP = new LoggedTunableNumber("Kicker/kicker_kP");
  public static final LoggedTunableNumber kicker_kS = new LoggedTunableNumber("Kicker/kicker_kS");
  public static final LoggedTunableNumber kicker_kV = new LoggedTunableNumber("Kicker/kicker_kV");

  public Kicker(KickerIO io) {
    this.io = io;
  }

  private double targetRPMSetpoint = 0.0;

  public void setKickerVoltage(double volts) {
    io.setKickerVoltage(volts);
  }

  public void setKickerVelocity(double rpm) {
    targetRPMSetpoint = rpm;
    io.setKickerVelocity(rpm);
  }

  public boolean kickerAtSetpoint() {
    final double toleranceRPM = 50.0;
    return Math.abs(targetRPMSetpoint - inputs.kickerRPM) <= toleranceRPM;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);

    if (Constants.tuningMode && 
        (kicker_kP.hasChanged(hashCode())
        || kicker_kS.hasChanged(hashCode())
        || kicker_kV.hasChanged(hashCode()))) {
      var kickerConfig = new SparkFlexConfig();
      kickerConfig.closedLoop.p(kicker_kP.get());
      io.updateKickerPID(kickerConfig);
      io.updateKickerFeedforward(kicker_kS.get(), kicker_kV.get());
    }
  }
}
