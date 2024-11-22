// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RumbleMyCat.RumbleCat;

public class bumblerumble extends Command {
  /** Creates a new bumblerumble. */
  RumbleCat cat;

  RumbleType catType;
  double strenghthCat;
  double strenghthCat2;
  Timer tim = new Timer();
  /**
   * @see Rumpble Rumble the controls
   * @param cat
   * @param catType
   * @param strenghthCat
   * @param strenghthCat2
   */
  public bumblerumble(
      RumbleCat cat, RumbleType catType, double strenghthCat, double strenghthCat2) {
    // Use addRequirements() here to declare subsystem dependencies. double
    this.cat = cat;
    this.catType = catType;
    this.strenghthCat = strenghthCat;
    this.strenghthCat2 = strenghthCat2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tim.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cat.setVibration(strenghthCat, catType);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cat.setVibration(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tim.get() > strenghthCat2;
  }
}
