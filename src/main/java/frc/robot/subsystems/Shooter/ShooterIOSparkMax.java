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

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class ShooterIOSparkMax implements ShooterIO {
  private static final double GEAR_RATIO = 1.0;

  private final CANSparkMax UpperShooter =
      new CANSparkMax(frc.robot.Constants.SHOOTER_TOP, MotorType.kBrushless);

  private final CANSparkMax LowerShooter =
      new CANSparkMax(frc.robot.Constants.SHOOTER_BOTTOM, MotorType.kBrushless);

  private final CANSparkMax Indexer =
      new CANSparkMax(frc.robot.Constants.INDEXER, MotorType.kBrushless);

  private final CANSparkMax UpperIntake =
      new CANSparkMax(frc.robot.Constants.INTAKE, MotorType.kBrushless);

  private final CANSparkMax LowerIntake =
      new CANSparkMax(frc.robot.Constants.INTAKE_BOTTOM, MotorType.kBrushless);

  private final RelativeEncoder US_encoder = UpperShooter.getEncoder();
  private final SparkPIDController US_pid = UpperShooter.getPIDController();

  private final RelativeEncoder LS_encoder = LowerShooter.getEncoder();
  private final SparkPIDController LS_pid = LowerShooter.getPIDController();

  private final RelativeEncoder I_encoder = Indexer.getEncoder();
  private final SparkPIDController I_pid = Indexer.getPIDController();

  private final RelativeEncoder UI_encoder = UpperIntake.getEncoder();
  private final SparkPIDController UI_pid = UpperIntake.getPIDController();

  private final RelativeEncoder LI_encoder = LowerIntake.getEncoder();
  private final SparkPIDController LI_pid = LowerIntake.getPIDController();

  public ShooterIOSparkMax() {
    UpperShooter.restoreFactoryDefaults();
    UpperShooter.setCANTimeout(250);
    UpperShooter.setInverted(false);
    UpperShooter.enableVoltageCompensation(11.0);
    UpperShooter.setSmartCurrentLimit(35);
    UpperShooter.burnFlash();

    LowerShooter.restoreFactoryDefaults();
    LowerShooter.setCANTimeout(250);
    LowerShooter.setInverted(true);
    LowerShooter.enableVoltageCompensation(11.0);
    LowerShooter.setSmartCurrentLimit(35);
    LowerShooter.burnFlash();

    Indexer.restoreFactoryDefaults();
    Indexer.setCANTimeout(250);
    Indexer.setInverted(true);
    Indexer.enableVoltageCompensation(11.0);
    Indexer.setSmartCurrentLimit(35);
    Indexer.burnFlash();

    UpperIntake.restoreFactoryDefaults();
    UpperIntake.setCANTimeout(250);
    UpperIntake.setInverted(false);
    UpperIntake.enableVoltageCompensation(11.0);
    UpperIntake.setSmartCurrentLimit(35);
    UpperIntake.burnFlash();

    LowerIntake.restoreFactoryDefaults();
    LowerIntake.setCANTimeout(250);
    LowerIntake.setInverted(false);
    LowerIntake.enableVoltageCompensation(11.0);
    LowerIntake.setSmartCurrentLimit(35);
    LowerIntake.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    inputs.US_positionRad = US_encoder.getPosition();
    inputs.US_velocityRPM = US_encoder.getVelocity();
    inputs.US_appliedVolts = UpperShooter.getAppliedOutput() * UpperShooter.getBusVoltage();
    inputs.US_currentAmps = UpperShooter.getOutputCurrent();

    inputs.LS_positionRad = LS_encoder.getPosition();
    inputs.LS_velocityRPM = LS_encoder.getVelocity();
    inputs.LS_appliedVolts = LowerShooter.getAppliedOutput() * LowerShooter.getBusVoltage();
    inputs.LS_currentAmps = LowerShooter.getOutputCurrent();

    inputs.I_positionRad = I_encoder.getPosition();
    inputs.I_velocityRPM = I_encoder.getVelocity();
    inputs.I_appliedVolts = Indexer.getAppliedOutput() * Indexer.getBusVoltage();
    inputs.I_currentAmps = Indexer.getOutputCurrent();

    inputs.UI_positionRad = UI_encoder.getPosition();
    inputs.UI_velocityRPM = UI_encoder.getVelocity();
    inputs.UI_appliedVolts = UpperIntake.getAppliedOutput() * UpperIntake.getBusVoltage();
    inputs.UI_currentAmps = UpperIntake.getOutputCurrent();

    inputs.LI_positionRad = LI_encoder.getPosition();
    inputs.LI_velocityRPM = LI_encoder.getVelocity();
    inputs.LI_appliedVolts = LowerIntake.getAppliedOutput() * LowerIntake.getBusVoltage();
    inputs.LI_currentAmps = LowerIntake.getOutputCurrent();
  }

  // @Override
  // public void setVoltage(double volts) {
  //   leader.setVoltage(volts);
  //   follower.setVoltage(volts);
  // }

  @Override
  public void setShooterVelocity(
      double velocityRadPerSec_Upper,
      double ffVolts_Upper,
      double velocityRadPerSec_Lower,
      double ffVolts_Lower) {

    US_pid.setReference(
        velocityRadPerSec_Upper, ControlType.kVelocity, 0, ffVolts_Upper, ArbFFUnits.kVoltage);
    LS_pid.setReference(
        velocityRadPerSec_Lower, ControlType.kVelocity, 0, ffVolts_Lower, ArbFFUnits.kVoltage);
  }

  @Override
  public void setIntakeVelocity(
      double velocityRadPerSec_Upper,
      double ffVolts_Upper,
      double velocityRadPerSec_Lower,
      double ffVolts_Lower) {
    UI_pid.setReference(velocityRadPerSec_Upper, ControlType.kVelocity, 0);
    LI_pid.setReference(velocityRadPerSec_Lower, ControlType.kVelocity, 0);
  }

  @Override
  public void setIndexerVelocity(double velocityRadPerSec_Upper, double ffVolts_Upper) {
    I_pid.setReference(velocityRadPerSec_Upper, ControlType.kVelocity, 0);
  }

  @Override
  public void stop() {
    UpperShooter.stopMotor();
    LowerShooter.stopMotor();
    Indexer.stopMotor();
    UpperIntake.stopMotor();
    LowerIntake.stopMotor();
  }

  @Override
  public void Shooter_configurePID(double kP, double kI, double kD) {
    US_pid.setP(kP, 0);
    US_pid.setI(kI, 0);
    US_pid.setD(kD, 0);
    US_pid.setFF(0, 0);

    LS_pid.setP(kP, 0);
    LS_pid.setI(kI, 0);
    LS_pid.setD(kD, 0);
    LS_pid.setFF(0, 0);
  }

  @Override
  public void Intake_configurePID(double kP, double kI, double kD) {
    UI_pid.setP(kP, 0);
    UI_pid.setI(kI, 0);
    UI_pid.setD(kD, 0);
    UI_pid.setFF(0, 0);

    LI_pid.setP(kP, 0);
    LI_pid.setI(kI, 0);
    LI_pid.setD(kD, 0);
    LI_pid.setFF(0, 0);
  }

  @Override
  public void Indexer_configurePID(double kP, double kI, double kD) {
    I_pid.setP(kP, 0);
    I_pid.setI(kI, 0);
    I_pid.setD(kD, 0);
    I_pid.setFF(0, 0);
  }
}
