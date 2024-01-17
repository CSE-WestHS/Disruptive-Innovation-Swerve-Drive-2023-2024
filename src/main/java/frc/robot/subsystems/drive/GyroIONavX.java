// Copyright 2021-2023 FRC 6328
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

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;

/** IO implementation for Pigeon2 */
public class GyroIONavX implements GyroIO {
  private final AHRS NavX = new AHRS(Port.kMXP);

  public GyroIONavX() {
    NavX.reset();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = NavX.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(-NavX.getYaw());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(NavX.getVelocityZ());
  }
}
