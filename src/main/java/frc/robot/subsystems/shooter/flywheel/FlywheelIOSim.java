// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.shooter.flywheel;

// import com.ctre.phoenix6.configs.Slot0Configs;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.wpilibj.simulation.FlywheelSim;

// public class FlywheelIOSim implements FlywheelIO {
//   private static final DCMotor flywheelMotorModel = DCMotor.getKrakenX60(9);

//   private double kS;
//   private double kV;
//   public double flywheelVolts;
//   public SimpleMotorFeedforward flywheelFeedforward;
//   private final PIDController velocityPID;

//   public FlywheelIOSim() {
//     flywheelVolts = 0.0;
//     kS = 0.0;
//     kV = 0.0;
//     flywheelFeedforward = new SimpleMotorFeedforward(kS, kV);
//     velocityPID = new PIDController(0, 0, 0);
//   }

//   private static final FlywheelSim flywheelSim =
//       new FlywheelSim(
//           LinearSystemId.createFlywheelSystem(flywheelMotorModel, .025, 1), flywheelMotorModel);

//   @Override
//   public void updateInputs(FlywheelIOInputs inputs) {
//     inputs.flywheelConnected = true;
//     inputs.flywheelAppliedVoltage = flywheelSim.getInputVoltage();
//     inputs.flywheelVelocityRPM = flywheelSim.getAngularVelocityRPM();
//   }

//   @Override
//   public void updateFlywheelPID(Slot0Configs config) {
//     velocityPID.setP(config.kP);
//   }

//   @Override
//   public void updateFlywheelFeedforward(double kS, double kV) {
//     this.kS = kS;
//     this.kV = kV;
//     flywheelFeedforward = new SimpleMotorFeedforward(this.kS, this.kV);
//   }

//   public void setFlywheelVoltage(double volts) {
//     this.flywheelVolts = MathUtil.clamp(volts, -12.0, 12.0);
//     flywheelSim.setInputVoltage(this.flywheelVolts);
//   }

//   @Override
//   public void setFlywheelVelocity(double targetRPM) {
//     double targetRotationsPerSec = targetRPM / 60;
//     double currentRotationsPerSec = flywheelSim.getAngularVelocityRPM() / 60;

//     // PID on velocity (rot/s)
//     double pidVolts = velocityPID.calculate(currentRotationsPerSec, targetRotationsPerSec);

//     // Use the ffVolts
//     double ffVolts = flywheelFeedforward.calculate(targetRotationsPerSec);

//     double appliedVolts = MathUtil.clamp(pidVolts + ffVolts, -12.0, 12.0);
//     flywheelSim.setInputVoltage(appliedVolts);
//   }
// }
