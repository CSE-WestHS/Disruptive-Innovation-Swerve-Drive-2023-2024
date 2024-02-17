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

// import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {

  private SingleJointedArmSim sim =
      new SingleJointedArmSim(DCMotor.getNEO(1), 1, 0.8, 0.8, 0.31, 0.83, false, 0.31);
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (closedLoop) {
      // appliedVolts =
      //     MathUtil.clamp(pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts, -12.0,
      // 12.0);
      // sim.setInputVoltage(appliedVolts);
    }

    sim.update(0.02);

    // inputs.positionRad = 0.0;
    // inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    // inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.position = (sim.getAngleRads()) * 180;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  // @Override
  // public void setVelocity(double velocityRadPerSec, double ffVolts) {
  //   closedLoop = true;
  //   pid.setSetpoint(velocityRadPerSec);
  //   this.ffVolts = ffVolts;
  // }
  @Override
  public void setPosition(double position) {
    pid.setSetpoint(position);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
