// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Arm.ArmAngleAmp;
import frc.robot.commands.Arm.ArmAngleSpeaker;
import frc.robot.commands.Arm.ArmDownGradual;
import frc.robot.commands.Arm.ArmSetAngle;
import frc.robot.commands.Arm.ArmUpGradual;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Indexer.AcquireNote;
import frc.robot.commands.Intake.ManualGrabNote;
import frc.robot.commands.Shooter.ShootNoteAmp;
import frc.robot.commands.Shooter.ShootNoteSpeaker;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmIO;
import frc.robot.subsystems.Arm.ArmIOSim;
import frc.robot.subsystems.Arm.ArmIOSparkMax;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIO;
import frc.robot.subsystems.Indexer.IndexerIOSim;
import frc.robot.subsystems.Indexer.IndexerIOSparkMax;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Intake.IntakeIOSparkMax;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.Shooter.ShooterIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  public final Indexer indexer;
  public final Intake intake;
  public final Shooter shooter;
  public final Arm arm;
  // public final LEDS leds;
  // public final Camera camera;

  // Controller
  private final CommandXboxController controllerDriver = new CommandXboxController(0);
  private final CommandXboxController controllerOperator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  // private final LoggedDashboardNumber flywheelSpeedInput =
  //   new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
        indexer = new Indexer(new IndexerIOSparkMax());
        intake = new Intake(new IntakeIOSparkMax());
        shooter = new Shooter(new ShooterIOSparkMax());
        arm = new Arm(new ArmIOSparkMax());
        // camera = new Camera();
        // leds = new LEDS();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        indexer = new Indexer(new IndexerIOSim());
        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new ShooterIOSim());
        arm = new Arm(new ArmIOSim());
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
        indexer = new Indexer(new IndexerIO() {});
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {});
        arm = new Arm(new ArmIO() {});
        break;
    }

    // Set up auto routines
    // NamedCommands.registerCommand(
    //  "Run Flywheel",
    // Commands.startEnd(
    //      () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
    // .withTimeout(5.0));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // // Set up feedforward characterization
    // autoChooser
    //.addOption(
    //     "Drive FF Characterization",
    //     new FeedForwardCharacterization(
    //         drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    // autoChooser.addOption("CenterBlue", new PathPlannerAuto("AllAutoCenterBlue"));
    // autoChooser.addOption("LeftBlue", new PathPlannerAuto("AllAutoLeftBlue"));
    // autoChooser.addOption("RightBlue", new PathPlannerAuto("AllAutoRightBlue"));
    // autoChooser.addOption("AutoScoreLeft", new PathPlannerAuto("ScoreAutoLeft"));

    // camera.useFrontCamera();
    // leds.RunLEDS();
    // autoChooser.addOption(
    //  "Flywheel FF Characterization",
    // new FeedForwardCharacterization(
    //  flywheel, flywheel::runVolts, flywheel::getCharacterizationVelocity));

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

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controllerDriver.getLeftY(),
            () -> -controllerDriver.getLeftX(),
            () -> controllerDriver.getRightX(),
            () -> controllerDriver.getLeftTriggerAxis()));

    // Driver Controls ********************************************************************
    /* Left Stick - Translate
     * Right Stick - Rotate
     *
     * Left Trigger - Turbo Drive Speed
     * Left Bumper - Start Intake
     *
     * Right Trigger - Score Amp Command
     * Right Bumper - Score Speaker Command
     *
     */

    controllerDriver.leftBumper().onTrue(new AcquireNote(indexer, intake));

    controllerDriver
        .rightBumper()
        .onTrue(new ArmAngleSpeaker(arm).andThen(new ShootNoteSpeaker(indexer, shooter, 100)));
    controllerDriver
        .rightTrigger()
        .onTrue(new ArmAngleAmp(arm).andThen(new ShootNoteAmp(indexer, shooter, 1000)));

    controllerDriver.a().onTrue(new ArmSetAngle(arm, Constants.ANGLE_CLIMB_UP));
    controllerDriver.x().onTrue(new ArmSetAngle(arm, Constants.ANGLE_CLIMB_DOWN));

    // Operator Controls ********************************************************************
    /*
        * a- reverse intake
        * b- reverse indexer
        * x- stop with x
        *
        *
        * d-up- gradual arm up
        * d-down - gradual arm down
        *
        *
        *
        * right trigger - run intake and indexer
        * right bumper - run shoter rollers
        *
        * left trigger - arm down
        * left bumper - arm up
        *
        * left little middle button - reset pose
        *
        *
        *     // controllerDriver.leftBumber().whileTrue(Commands.run(() -> intake.runVelocity(2200)));
    controllerDriver.povUp().onTrue(new ArmUpGradual(arm));
       controllerDriver.povDown().onTrue(new ArmDownGradual(arm));
        *
        *
        */

    // Reset pose, needs to be little left middle button
    controllerOperator
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controllerOperator.povUp().onTrue(new ArmUpGradual(arm));
    controllerOperator.povDown().onTrue(new ArmDownGradual(arm));

    controllerOperator.a().whileTrue(Commands.run(() -> intake.runVelocity(-2200)));

    controllerOperator.x().onTrue(Commands.run(() -> drive.stopWithX()));
    controllerOperator.rightTrigger().whileTrue(new ManualGrabNote(intake, indexer, 2000));
    controllerOperator.rightBumper().whileTrue(Commands.run(() -> shooter.runVelocity(4000)));
    controllerOperator.leftTrigger().onTrue(new ArmAngleSpeaker(arm));
    controllerOperator.leftBumper().onTrue(new ArmAngleAmp(arm));
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
