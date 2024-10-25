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

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double US_positionRad = 0.0;
    public double US_velocityRPM = 0.0;
    public double US_appliedVolts = 0.0;
    public double US_currentAmps = 0.0;

    public double LS_positionRad = 0.0;
    public double LS_velocityRPM = 0.0;
    public double LS_appliedVolts = 0.0;
    public double LS_currentAmps = 0.0;

    public double UI_positionRad = 0.0;
    public double UI_velocityRPM = 0.0;
    public double UI_appliedVolts = 0.0;
    public double UI_currentAmps = 0.0;

    public double LI_positionRad = 0.0;
    public double LI_velocityRPM = 0.0;
    public double LI_appliedVolts = 0.0;
    public double LI_currentAmps = 0.0;

    public double I_positionRad = 0.0;
    public double I_velocityRPM = 0.0;
    public double I_appliedVolts = 0.0;
    public double I_currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  // /** Run open loop at the specified voltage. */
  // public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setShooterVelocity(
      double velocityRadPerSec_Upper,
      double ffVolts_Upper,
      double velocityRadPerSec_Lower,
      double ffVolts_Lower) {}

  public default void setIntakeVelocity(
      double velocityRadPerSec_Upper,
      double ffVolts_Upper,
      double velocityRadPerSec_Lower,
      double ffVolts_Lower) {}

  public default void setIndexerVelocity(double velocityRadPerSec_Upper, double ffVolts_Upper) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set velocity PID constants. */
  public default void Shooter_configurePID(double kP, double kI, double kD) {}

  public default void Intake_configurePID(double kP, double kI, double kD) {}

  public default void Indexer_configurePID(double kP, double kI, double kD) {}
}
