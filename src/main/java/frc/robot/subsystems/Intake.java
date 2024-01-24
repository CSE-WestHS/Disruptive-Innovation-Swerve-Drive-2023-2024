package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake {
     private final CANSparkMax intakeMotor1 = new CANSparkMax(13, MotorType.kBrushless);
  private final CANSparkMax intakeMotor2 = new CANSparkMax(14, MotorType.kBrushless);
  public Intake(){
    intakeMotor1.clearFaults();
    intakeMotor2.clearFaults();
    intakeMotor1.setSmartCurrentLimit(40);
    intakeMotor2.setSmartCurrentLimit(40);
    //mimics the other claw motor, as they do the same thing.
    intakeMotor2.follow(intakeMotor1, false);
    intakeMotor1.set(0);
  }
  
  public void runClaw(double speed){
    intakeMotor1.set(speed);
  }

  public void stopClaw(){
    intakeMotor1.set(0);
  }
}
}
