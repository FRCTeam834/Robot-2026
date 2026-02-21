// Copyright (c) FIRST and other
// WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public static enum RollerState{
    FAST(8.0),
    SLOW(3.0),
    REVERSE (-3.0),
    STOP(0.0);

    public final double voltage;

    private RollerState (double voltage){
      this.voltage = voltage;
    }
  };

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
    if (Constants.tuningMode && 
        (pivot_kP.hasChanged(hashCode())
        || pivot_kS.hasChanged(hashCode())
        || pivot_kV.hasChanged(hashCode())
        || pivot_kG.hasChanged(hashCode()))) {
      var pivotConfig = new SparkMaxConfig();
      pivotConfig.closedLoop.p(pivot_kP.get());
      io.updatePivotPID(pivotConfig);
      io.updatePivotFeedforward(pivot_kS.get(), pivot_kG.get(), pivot_kV.get());
    }

    // Roller
    if (Constants.tuningMode && 
        (roller_kP.hasChanged(hashCode())
        || roller_kS.hasChanged(hashCode())
        || roller_kV.hasChanged(hashCode()))) {
      var rollerConfig = new SparkFlexConfig();
      rollerConfig.closedLoop.p(roller_kP.get());
      io.updateRollerPID(rollerConfig);
      io.updateRollerFeedforward(roller_kV.get(), roller_kS.get());
    }
  }

 // Roller Setter Methods
  public void setRollerState(RollerState intakeState){
    setRollerVoltage(intakeState.voltage);
  }
  public void setRollerRPM(double targetRPM) {    
    io.setRollerRPM(targetRPM); 
  }
   public void setRollerVoltage(double targetVolts) {
    io.setRollerVoltage(targetVolts); 
  }

  // Pivot Setter Methods
  public void setPivotVoltage(double targetVolts) {
    io.setPivotVoltage(targetVolts);
  }
  public void setPivotPosition(double targetPosition) {
    io.setPivotPosition(targetPosition);
  }

  // Roller Getter Methods
  public double getCurrentRollerVelocity(){
    return inputs.rollerRPM;
  }
  public double getCurrentRollerVoltage(){
    return inputs.rollerAppliedVoltage;    
  }

  // Pivot Getter Methods
  public double getCurrentPivotPosition(){
    return inputs.pivotPositionRads;
  }
  public double getCurrentPivotVoltage(){
    return inputs.pivotAppliedVoltage;    
  }

  // Roller Miscellaneous Methods
  public boolean rollerAtSetpoint(double targetRPM) {       
    final double toleranceRPM = 50.0;
    return Math.abs(targetRPM - inputs.rollerRPM) <= toleranceRPM;
  }
  public void stopMotors() {
    io.stopMotors();
  }

}
