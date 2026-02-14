package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class IntakeIOSpark implements IntakeIO {
  // Roller
  private SparkFlex rollerMotor;
  public SparkAbsoluteEncoder rollerEncoder;
  private double rollerVolts;
  private double rollerVelocity;
  private SparkFlexConfig rollerConfig;
  private SimpleMotorFeedforward rollerFeedforward;
  private SparkClosedLoopController rollerController; 

  // Pivot
  private SparkMax pivotMotor;
  public SparkAbsoluteEncoder pivotEncoder;
  private SparkMaxConfig pivotConfig;
  private double pivotVolts;
  private SimpleMotorFeedforward pivotFeedforward;

  public IntakeIOSpark() {
    // Roller
    rollerMotor = new SparkFlex(11, MotorType.kBrushless);
    rollerEncoder = rollerMotor.getAbsoluteEncoder();
    rollerVolts = rollerMotor.getBusVoltage();
    rollerVelocity = rollerEncoder.getVelocity();
    rollerConfig = new SparkFlexConfig();
    rollerFeedforward = new SimpleMotorFeedforward(0,0);
    rollerController = rollerMotor.getClosedLoopController();

    // Pivot
    pivotMotor = new SparkMax(12, MotorType.kBrushless);
    pivotEncoder = pivotMotor.getAbsoluteEncoder();
    pivotVolts = pivotMotor.getBusVoltage();
    pivotFeedforward = new SimpleMotorFeedforward(0,0);
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
  @Override
  public void setRollerVoltage(double volts) {
    this.rollerVolts = MathUtil.clamp(volts, -12.0, 12.0);
    rollerMotor.setVoltage(this.rollerVolts);
  }
  @Override
  public void setRollerRPM(double targetRPM) {
    rollerController.setSetpoint(targetRPM, null);
  }

  @Override
  public void updateRollerPID(SparkFlexConfig rollerMaxConfig) {
    this.rollerConfig = rollerMaxConfig;
    rollerConfig.closedLoop.p(0.1);
    rollerConfig.apply(rollerMaxConfig);
  }

  @Override
  public void updateRollerFeedforward(double kS, double kV){
    this.rollerFeedforward = new SimpleMotorFeedforward(kS, kV);
  }

  // Pivot Methods
  @Override
  public void setPivotVoltage(double volts) {
    this.pivotVolts = MathUtil.clamp(volts, -12.0, 12.0);
    pivotMotor.setVoltage(this.pivotVolts);
  }

  @Override
  public void updatePivotPID(SparkMaxConfig pivotConfig) {
    pivotMotor.configure(
        pivotConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
  }

  @Override
  public void updatePivotFeedForward(double kS, double kV) {
    pivotFeedforward = new SimpleMotorFeedforward(kS, kV);
  }
}
