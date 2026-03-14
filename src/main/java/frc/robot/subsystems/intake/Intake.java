// Copyright (c) FIRST and other
// WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.PivotState;
import frc.robot.subsystems.intake.IntakeConstants.RollerState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public static final LoggedTunableNumber pivot_kP = new LoggedTunableNumber("Intake/pivot_kP");
  public static final LoggedTunableNumber pivot_kS = new LoggedTunableNumber("Intake/pivot_kS");
  public static final LoggedTunableNumber pivot_kV = new LoggedTunableNumber("Intake/pivot_kV");

  private final PIDController pivotController = new PIDController(2, 0, 0);

  @AutoLogOutput(key = "SubsystemStates/rollerState")
  private RollerState rollerState;

  @AutoLogOutput(key = "SubsystemStates/pivotState")
  private PivotState pivotState;

  private boolean isPivotZeroed;

  static {
    pivot_kP.initDefault(0);
    pivot_kS.initDefault(0);
    pivot_kV.initDefault(0);
  }

  public Intake(IntakeIO io) {
    this.io = io;
    rollerState = RollerState.STOP;
    pivotState = PivotState.OFF;
    isPivotZeroed = false;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Pivot Tuning
    if (Constants.TUNING_MODE
        && (pivot_kP.hasChanged(hashCode())
            || pivot_kS.hasChanged(hashCode())
            || pivot_kV.hasChanged(hashCode()))) {

      var pivotConfig = new SparkFlexConfig();
      pivotConfig.closedLoop.p(pivot_kP.get());
      pivotConfig.closedLoop.feedForward.kS(pivot_kS.get()).kV(pivot_kV.get());

      io.updateClosedLoopConfig(pivotConfig);
    }

    // handle the transition from deploying to deployed

    if (pivotState == PivotState.DEPLOYING
        && pivotAtSetpoint(PivotState.DEPLOYING, IntakeConstants.pivotTolerance)) {
      stopPivot();
      pivotState = PivotState.DEPLOYED;
    }

    switch (pivotState) {
      case DEPLOYING -> io.setPivotVoltage(
          pivotController.calculate(getCurrentPivotAngle(), PivotState.DEPLOYING.position));
      case UP -> io.setPivotVoltage(
          pivotController.calculate(getCurrentPivotAngle(), PivotState.UP.position));
      case STOW -> io.setPivotVoltage(
          pivotController.calculate(getCurrentPivotAngle(), PivotState.STOW.position));
      case DEPLOYED -> io.setPivotVoltage(0.1);
      case OFF -> {}
    }

    setRollerVoltage(rollerState.voltage);
  }

  // Roller Setter Methods
  public void setDesiredRollerState(RollerState rollerState) {
    this.rollerState = rollerState;
    setRollerVoltage(rollerState.voltage);
  }

  public void setRollerVoltage(double targetVolts) {
    io.setRollerVoltage(targetVolts);
  }

  // Pivot Setter Methods
  public void setDesiredPivotState(PivotState pivotState) {
    this.pivotState = pivotState;
  }

  public void setPivotVoltage(double targetVolts) {
    io.setPivotVoltage(targetVolts);
  }

  public boolean DoesPivotNeedZero() {
    return !isPivotZeroed;
  }

  public void establishPivotZero(boolean state) {
    isPivotZeroed = state;
  }

  public void setEncoderAngle(double angle) {
    io.setEncoderAngle(angle);
  }

  // Roller Getter Methods
  public double getCurrentRollerVelocity() {
    return inputs.rollerRPM;
  }

  public double getCurrentRollerVoltage() {
    return inputs.rollerAppliedVoltage;
  }

  // Pivot Getter Methods
  public double getCurrentPivotAngle() {
    return inputs.pivotPositionRads;
  }

  public double getCurrentPivotVoltage() {
    return inputs.pivotAppliedVoltage;
  }

  public double getPivotVelocity() {
    return inputs.pivotVelocity;
  }

  public void setPivotAngle(double angle) {
    io.setPivotAngle(angle);
  }

  public double getPivotCurrent() {
    return inputs.pivotCurrent;
  }

  public PivotState getPivotState() {
    return pivotState;
  }

  public RollerState getRollersState() {
    return rollerState;
  }

  public boolean pivotAtSetpoint(PivotState state, double toleranceRads) {
    return MathUtil.isNear(state.position, getCurrentPivotAngle(), toleranceRads);
  }

  public void stopPivot() {
    io.stopPivot();
  }

  public void stopRollers() {
    io.stopRollers();
  }
}
