package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase { 
  private static Shooter mShooter; 
  private final CANSparkMax top_motor, down_motor;
  
  private Shooter() {
    top_motor = new CANSparkMax(0, MotorType.kBrushless); 
    down_motor = new CANSparkMax(0, MotorType.kBrushless); 
    top_motor.setIdleMode(IdleMode.kBrake); 
    down_motor.setIdleMode(IdleMode.kBrake); 
  } 

  public static Shooter getInstance () { 
    if (mShooter == null) { 
      mShooter = new Shooter(); 
    }
    return mShooter; 
  }

  @Override
  public void periodic() {}

  public void setSpeed (double output) {
    top_motor.set(output); 
    down_motor.set(output); 
  }
}
