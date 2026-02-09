package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

public class IntakeIOSparkMax implements IntakeIO {
  // Roller
  private SparkMax rollerMotor;
  private SparkAbsoluteEncoder rollerEncoder;
  private SparkMaxConfig rollerConfig;
  private double rollerVolts;
  private double rollerVelocity = rollerEncoder.getVelocity();

  // Pivot
  private SparkMax pivotMotor;
  private SparkAbsoluteEncoder pivotEncoder;
  private SparkMaxConfig pivotConfig;
  private double pivotVolts;
  private double pivotVelocity;

  public IntakeIOSparkMax() {
    // Roller
    rollerMotor = new SparkMax(11, MotorType.kBrushless);
    rollerEncoder = rollerMotor.getAbsoluteEncoder();
    rollerVolts = rollerMotor.getBusVoltage();
    rollerVelocity

        // Pivot
        pivotMotor = new SparkMax(12, MotorType.kBrushless);
    pivotEncoder = pivotMotor.getAbsoluteEncoder();
    pivotVolts = pivotMotor.getBusVoltage();
    pivotVelocity = pivotEncoder.getVelocity();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerConnected = true;
    inputs.rollerVelocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rollerVelocity);
    inputs.rollerAppliedVoltage = rollerVolts;

    inputs.pivotConnected = true;
    inputs.pivotVelocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(pivotVelocity);
    inputs.pivotPositionRads = pivotEncoder.getPosition();
    inputs.pivotAppliedVoltage = pivotVolts;
  }
  // Roller Methods
  public void setRollerVoltage(double volts) {
    this.rollerVolts = MathUtil.clamp(volts, -12.0, 12.0);
    rollerMotor.setRollerVoltage(this.rollerVolts);
  }

  // Pivot Methods
  public void setPivotVoltage(double volts) {
    this.pivotVolts = MathUtil.clamp(volts, -12.0, 12.0);
    pivotMotor.setPivotVoltage(this.pivotVolts);
  }

  public void updateRollerPID(SparkMaxConfig rollerConfig, double kS, double kV) {
    this.rollerConfig = rollerConfig;
    rollerConfig.configure(
        rollerConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.ResetMode.kNoPersistParameters);
  }

  public void updatePivotPID(SparkMaxConfig pivotConfig, double kS, double kV) {
    this.pivotConfig = pivotConfig;
    pivotConfig.configure(
        pivotConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.ResetMode.kNoPersistParameters);
  }
}
