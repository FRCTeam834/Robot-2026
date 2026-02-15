package frc.robot.subsystems.intake;

import com.revrobotics.spark.ClosedLoopSlot;
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
  private SparkAbsoluteEncoder rollerEncoder;
  private double rollerVolts;
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
    rollerConfig = new SparkFlexConfig();
    rollerFeedforward = new SimpleMotorFeedforward(0,0);
    rollerController = rollerMotor.getClosedLoopController();

    // Pivot
    pivotMotor = new SparkMax(12, MotorType.kBrushless);
    pivotEncoder = pivotMotor.getAbsoluteEncoder();
    pivotVolts = pivotMotor.getBusVoltage();
    pivotFeedforward = new SimpleMotorFeedforward(0,0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Roller
    inputs.rollerConnected = true;
    inputs.rollerRPM = rollerEncoder.getVelocity();
    inputs.rollerAppliedVoltage = rollerVolts;

    // Pivot
    inputs.pivotConnected = true;
    inputs.pivotPositionRads = pivotEncoder.getPosition();
    inputs.pivotAppliedVoltage = pivotVolts;
  }

  // Roller Methods
  @Override
  public void setRollerVoltage(double volts) {
    this.rollerVolts = MathUtil.clamp(volts, -12.0, 12.0);
    rollerMotor.setVoltage(this.rollerVolts);
  }

  @Override
  public void setRollerRPM(double targetRPM) {
    double targetRotationsPerSec = targetRPM / 60;
    double ffVolts = rollerFeedforward.calculate(targetRotationsPerSec);
    rollerController.setSetpoint(targetRPM, SparkFlex.ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts);  
  }

  @Override
  public void updateRollerPID(SparkFlexConfig config) {
    this.rollerConfig = config;
    rollerMotor.configure(rollerConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
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
  public void updatePivotPID(SparkMaxConfig config) {
    this.pivotConfig = config;
    pivotMotor.configure(pivotConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
  }

  @Override
  public void updatePivotFeedforward(double kS, double kV) {
    this.pivotFeedforward = new SimpleMotorFeedforward(kS, kV);
  }
}
