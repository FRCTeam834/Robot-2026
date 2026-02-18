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
import edu.wpi.first.math.controller.ArmFeedforward;



public class IntakeIOSpark implements IntakeIO {
  // Roller
  private SparkFlex rollerMotor;
  private SparkAbsoluteEncoder rollerEncoder;
  private SparkFlexConfig rollerConfig;
  private SimpleMotorFeedforward rollerFeedforward;
  private SparkClosedLoopController rollerController;

  // Pivot
  private SparkMax pivotMotor;
  public SparkAbsoluteEncoder pivotEncoder;
  private SparkMaxConfig pivotConfig;
  private double pivotVolts;
  private ArmFeedforward pivotFeedforward;
  private SparkClosedLoopController pivotController;

  public IntakeIOSpark() {
    // Roller
    rollerMotor = new SparkFlex(11, MotorType.kBrushless);
    rollerEncoder = rollerMotor.getAbsoluteEncoder();
    rollerConfig = new SparkFlexConfig();
    rollerFeedforward = new SimpleMotorFeedforward(0, 0);
    rollerController = rollerMotor.getClosedLoopController();

    // Pivot
    pivotMotor = new SparkMax(12, MotorType.kBrushless);
    pivotEncoder = pivotMotor.getAbsoluteEncoder();
    pivotConfig = new SparkMaxConfig();
    pivotFeedforward = new ArmFeedforward(0, 0, 0);
    pivotController = pivotMotor.getClosedLoopController();
  }


  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Roller
    inputs.rollerConnected = true;
    inputs.rollerRPM = rollerEncoder.getVelocity();
    inputs.rollerAppliedVoltage = rollerMotor.getBusVoltage();

    // Pivot
    inputs.pivotConnected = true;
    inputs.pivotPositionRads = pivotEncoder.getPosition() * 2 * Math.PI; //see if right
    inputs.pivotAppliedVoltage = pivotMotor.getBusVoltage();
    inputs.pivotRPM = pivotEncoder.getVelocity();

  }

  // Roller Methods
  @Override
  public void setRollerVoltage(double targetVolts) {
    rollerMotor.setVoltage(MathUtil.clamp(targetVolts, -12.0, 12.0));
  }

  @Override
  public void setRollerRPM(double targetRPM) {
    double targetRotationsPerSec = targetRPM / 60;
    double ffVolts = rollerFeedforward.calculate(targetRotationsPerSec);
    rollerController.setSetpoint(
        targetRPM, SparkFlex.ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts);
  }

  @Override
  public void updateRollerPID(SparkFlexConfig config) {
    this.rollerConfig = config;
    rollerMotor.configure(
        rollerConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateRollerFeedforward(double kS, double kV) {
    this.rollerFeedforward = new SimpleMotorFeedforward(kS, kV);
  }

  // Pivot Methods
  @Override
  public void setPivotVoltage(double volts) {
    this.pivotVolts = MathUtil.clamp(volts, -12.0, 12.0);
    pivotMotor.setVoltage(this.pivotVolts);
  }
  
  @Override
  public void setPivotPosition(double targetPositionRads, double pivotRPM, double pivotRadsOffset) {
    double adjustedPositionRads = targetPositionRads + pivotRadsOffset;
    double adjustedPositionRotations = adjustedPositionRads / (2.0 * Math.PI);
    double pivotRadsPerSec = pivotRPM * (2.0 * Math.PI) / 60.0; 
    double ffVolts = pivotFeedforward.calculate(adjustedPositionRads, pivotRadsPerSec);

    pivotController.setSetpoint(
        adjustedPositionRotations, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVolts);
  }

  @Override
  public void updatePivotPID(SparkMaxConfig config) {
    this.pivotConfig = config;
    pivotMotor.configure(
        pivotConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
  }

  // Make it an armfeedforward
  @Override
  public void updatePivotFeedforward(double kS, double kG, double kV) {
    this.pivotFeedforward = new ArmFeedforward(kS, kG, kV);
  }
}
