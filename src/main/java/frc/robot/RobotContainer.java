// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.commands.intake.ZeroIntake;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSparkFlex;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.kicker.Kicker;
import frc.robot.subsystems.shooter.kicker.KickerConstants.KickerState;
import frc.robot.subsystems.shooter.kicker.KickerIO;
import frc.robot.subsystems.shooter.kicker.KickerIOSpark;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public static Drive drive;
  public static Vision vision;
  public static Indexer indexer;
  public static Intake intake;
  public static Flywheel flywheel;
  public static Kicker kicker;

  public static final CommandXboxController DRIVE_CONTROLLER = new CommandXboxController(0);
  public static final CommandXboxController OPERATOR_CONTROLLER = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOLimelight("limelight-front", drive::getRotation),
        //         new VisionIOLimelight("limelight-right", drive::getRotation));

        indexer = new Indexer(new IndexerIOSparkFlex());
        intake = new Intake(new IntakeIOSpark());
        flywheel = new Flywheel(new FlywheelIOTalonFX(), drive::getDistanceToHub);
        kicker = new Kicker(new KickerIOSpark());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        indexer = new Indexer(new IndexerIO() {});
        intake = new Intake(new IntakeIO() {});
        flywheel = new Flywheel(new FlywheelIO() {}, drive::getDistanceToHub);
        kicker = new Kicker(new KickerIO() {});

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    NamedCommands.registerCommand(
        "hubshot",
        ShooterCommands.shootWhenReadyManualVelocity(1530, flywheel, kicker, intake, indexer));
    NamedCommands.registerCommand("pivotdown", IntakeCommands.deployIntake);
    NamedCommands.registerCommand("zerointake", new ZeroIntake(intake));
    NamedCommands.registerCommand("rampup1700", ShooterCommands.rampToRPM(1700, flywheel));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption("centerbuns", new PathPlannerAuto("centerbuns"));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // flywheel.setDefaultCommand(
    //     ShooterCommands.dumbFlywheel(OPERATOR_CONTROLLER::getRightY, flywheel));

    // * Default command just a plain drive */
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -DRIVE_CONTROLLER.getLeftY(),
            () -> -DRIVE_CONTROLLER.getLeftX(),
            () -> -DRIVE_CONTROLLER.getRightX(),
            () -> false));

    // Reset gyro to 0° when povdown button is pressed
    DRIVE_CONTROLLER
        .povDown()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero));
                    },
                    drive)
                .ignoringDisable(true));

    DRIVE_CONTROLLER
        .x()
        .whileTrue(
            Commands.run(
                    () -> {
                      kicker.setDesiredState(KickerState.FEED);
                    },
                    kicker)
                .finallyDo(() -> kicker.setDesiredState(KickerState.STOP)));

    OPERATOR_CONTROLLER.a().whileTrue(new ZeroIntake(intake));
    OPERATOR_CONTROLLER.povDown().onTrue(IntakeCommands.deployIntake);
    OPERATOR_CONTROLLER.povUp().onTrue(IntakeCommands.retractIntake);

    OPERATOR_CONTROLLER.x().onTrue(IntakeCommands.toggleFastRollers);

    OPERATOR_CONTROLLER.rightTrigger().onTrue(ShooterCommands.rampToRPM(1700, flywheel));
    OPERATOR_CONTROLLER.leftTrigger().onTrue(ShooterCommands.rampToRPM(1550, flywheel));

    OPERATOR_CONTROLLER
        .y()
        .whileTrue(
            Commands.run(() -> kicker.setDesiredState(KickerState.FEED))
                .finallyDo(() -> kicker.setDesiredState(KickerState.STOP)));

    DRIVE_CONTROLLER
        .rightTrigger()
        .whileTrue(
            ShooterCommands.shootWhenReadyManualVelocity(1700, flywheel, kicker, intake, indexer));

    DRIVE_CONTROLLER
        .leftTrigger()
        .whileTrue(
            ShooterCommands.shootWhenReadyManualVelocity(1530, flywheel, kicker, intake, indexer));

    new JoystickButton(DRIVE_CONTROLLER.getHID(), 8).onTrue(IntakeCommands.getJoltIntake());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
