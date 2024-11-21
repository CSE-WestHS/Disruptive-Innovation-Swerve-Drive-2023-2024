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

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final SimpleMotorFeedforward Shooter_ffModel;
  private final SimpleMotorFeedforward Indexer_ffModel;
  private final SimpleMotorFeedforward Intake_ffModel;
  // private final SysIdRoutine sysId;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    this.io = io;

    Shooter_ffModel = new SimpleMotorFeedforward(0.0051, 0.0017);
    io.Shooter_configurePID(.00009, 0.0000001, 0.0001);

    Intake_ffModel = new SimpleMotorFeedforward(0.001, 0.005);
    io.Intake_configurePID(.00003, 0.000001, 0.00);

    Indexer_ffModel = new SimpleMotorFeedforward(0.001, 0.005);
    io.Indexer_configurePID(.00005, 0.000001, 0.000);

    // Configure SysId
    // sysId =
    //     new SysIdRoutine(
    //         new SysIdRoutine.Config(
    //             null,
    //             null,
    //             null,
    //             (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
    //         new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  // /** Run open loop at the specified voltage. */
  // public void runVolts(double volts) {
  //   io.setVoltage(volts);
  // }

  /** Run closed loop at the specified velocity. */
  public void runShooterVelocity(double velocityRPM) {
    io.setShooterVelocity(
        velocityRPM,
        Shooter_ffModel.calculate(velocityRPM),
        velocityRPM,
        Shooter_ffModel.calculate(velocityRPM));

    Logger.recordOutput("Shooter/ShooterSetpointRPM", velocityRPM);
  }

  public void runIntakeVelocity(double velocityRPM) {
    io.setIntakeVelocity(
        velocityRPM,
        Intake_ffModel.calculate(velocityRPM),
        velocityRPM,
        Intake_ffModel.calculate(velocityRPM));
    Logger.recordOutput("Shooter/IntakeSetpointRPM", velocityRPM);
  }

  public void runIndexerVelocity(double velocityRPM) {
    io.setIndexerVelocity(velocityRPM, Indexer_ffModel.calculate(velocityRPM));
    Logger.recordOutput("Shooter/IndexerSetpointRPM", velocityRPM);
  }

  /** Stops the shooter. */
  public void stop() {
    io.stop();
  }

  // /** Returns a command to run a quasistatic test in the specified direction. */
  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return sysId.quasistatic(direction);
  // }

  // /** Returns a command to run a dynamic test in the specified direction. */
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return sysId.dynamic(direction);
}

  // /** Returns the current velocity in RPM. */
  // @AutoLogOutput
  // public double getVelocityRPM() {
  //   return inputs.velocityRPM;
  // }

  // /** Returns the current velocity in radians per second. */
  // public double getCharacterizationVelocity() {
  //   return inputs.velocityRPM;
  // }
// }
