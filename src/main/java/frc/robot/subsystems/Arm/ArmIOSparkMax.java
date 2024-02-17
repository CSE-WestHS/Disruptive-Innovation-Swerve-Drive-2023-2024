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

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkBase;
// import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
// import com.revrobotics.SparkPIDController.ArbFFUnits;
// import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class ArmIOSparkMax implements ArmIO {

  private final CANSparkMax leader =
      new CANSparkMax(frc.robot.Constants.LEFT_WORM_GEAR, MotorType.kBrushless);
  private final CANSparkMax follower =
      new CANSparkMax(frc.robot.Constants.RIGHT_WORM_GEAR, MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  private SparkPIDController pid = leader.getPIDController();

  public ArmIOSparkMax() {
    leader.restoreFactoryDefaults();
    follower.restoreFactoryDefaults();

    pid = leader.getPIDController();
    pid.setOutputRange(55, 136);

    leader.setCANTimeout(250);
    follower.setCANTimeout(250);

    leader.setInverted(false);
    follower.follow(leader, false);

    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(30);
    follower.enableVoltageCompensation(12.0);
    follower.setSmartCurrentLimit(30);

    encoder.setPositionConversionFactor(Constants.ARM_GEAR_RATIO);

    leader.burnFlash();
    follower.burnFlash();

    encoder.setPosition(Constants.ANGLE_SPEAKER);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    // inputs.velocityRadPerSec =
    //     Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent()};
    inputs.position = leader.getEncoder().getPosition();
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  /*@Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }
  */
  @Override
  public void setPosition(double position) {
    pid.setReference(position, CANSparkBase.ControlType.kPosition, 0);
    System.out.println(position);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }
}
