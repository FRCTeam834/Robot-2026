// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.studica.frc.Navx;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Queue;
import java.util.function.DoubleSupplier;

public class GyroIONavx3 extends SubsystemBase implements GyroIO {
  private final Navx navx;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavx3() {
    navx = new Navx(20, 100);

    int err = navx.enableOptionalMessages(true, true, true, true, true, true, true, true, true, true);
    if (err == 1) System.out.println("!!NAVX3 MESSAGE ENABLE ERROR!!");

    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(getYawDegrees());
  }

  public DoubleSupplier getYawDegrees() {
    return () -> navx.getYaw().in(Degrees);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = Rotation2d.fromDegrees(navx.getYaw().in(Degrees));
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(navx.getAngularVel()[2].in(DegreesPerSecond));

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
