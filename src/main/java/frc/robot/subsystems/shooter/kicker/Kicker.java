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

  public static enum KickerState{
    FAST(8.0),
    SLOW(3.0),
    REVERSE (-3.0),
    STOP(0.0);

    public final double voltage;

    private KickerState (double voltage){
      this.voltage = voltage;
    }
  };

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
  
  // Kicker Setter Methods
  public void setKickerVoltage(double volts) {
    io.setKickerVoltage(volts);
  }
  public void setKickerVelocity(double targetRPM) {       
    io.setKickerVelocity(targetRPM);
  }
  public void setKickerState(KickerState kickerState){
    setKickerVoltage(kickerState.voltage);
  }

  // Kicker Getter Methods
  public double getKickerVoltage(){
    return inputs.kickerAppliedVoltage;
  }
  public double getKickerRPM(){
    return inputs.kickerRPM;
  }

  // Miscellaneous Methods
  public boolean kickerAtSetpoint(double targetRPM) {     
    final double toleranceRPM = 50.0;
    return Math.abs(targetRPM - inputs.kickerRPM) <= toleranceRPM;
  }
  public void stopMotor() {
    io.stopMotor();
  }

}
