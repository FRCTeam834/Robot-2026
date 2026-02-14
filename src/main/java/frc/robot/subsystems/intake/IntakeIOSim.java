package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

public class IntakeIOSim implements IntakeIO {
  // Roller
  private SparkFlex rollerMotor;
  private SparkAbsoluteEncoder rollerEncoder;
  private double rollerVolts;
  private double rollerVelocity;
  private SparkFlexConfig rollerConfig;
  private SparkClosedLoopController rollerPID;

  // Pivot
  private SparkMax pivotMotor;
  private SparkAbsoluteEncoder pivotEncoder;
  private SparkMaxConfig pivotConfig;
  private double pivotVolts;
  // private double pivotVelocity;

  public IntakeIOSim() {
    // Roller
    rollerMotor = new SparkFlex(11, MotorType.kBrushless);
    rollerEncoder = rollerMotor.getAbsoluteEncoder();
    rollerVolts = rollerMotor.getBusVoltage();
    rollerVelocity = rollerEncoder.getVelocity();
    rollerPID = rollerMotor.getClosedLoopController();
    rollerConfig = new SparkFlexConfig();

    // Pivot
    pivotMotor = new SparkMax(12, MotorType.kBrushless);
    pivotEncoder = pivotMotor.getAbsoluteEncoder();
    pivotVolts = pivotMotor.getBusVoltage();
    // pivotVelocity = pivotEncoder.getVelocity();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Roller
    inputs.rollerConnected = true;
    inputs.rollerRPM = rollerVelocity;
    inputs.rollerAppliedVoltage = rollerVolts;

    // Pivot
    inputs.pivotConnected = true;
    inputs.pivotPositionRads = pivotEncoder.getPosition();
    inputs.pivotAppliedVoltage = pivotVolts;
    // inputs.pivotAppliedVelocity = pivotEncoder.getVelocity();
    // inputs.pivotVelocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(pivotVelocity);
  }

  // Roller Methods
  public void setRollerVoltage(double volts) {
    this.rollerVolts = MathUtil.clamp(volts, -12.0, 12.0);
    rollerMotor.setVoltage(this.rollerVolts);
  }

  public void setRollerRPM(double targetRPM, double ffVolts) {
    rollerPID.setSetpoint(targetRPM, null);
  }

  public void updateRollerPID(SparkFlexConfig rollerConfig, double kS, double kV) {
    this.rollerConfig = rollerConfig;
    rollerConfig.apply(rollerConfig);
  }

  // Pivot Methods
  public void setPivotVoltage(double volts) {
    this.pivotVolts = MathUtil.clamp(volts, -12.0, 12.0);
    pivotMotor.setVoltage(this.pivotVolts);
  }

  public void updatePivotPID(SparkMaxConfig pivotMaxConfig, double kS, double kG) {
    this.pivotConfig = pivotMaxConfig;
    pivotMotor.configure(
        pivotConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
  }
}
