package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase { 
  private static Shooter mShooter; 
  private final CANSparkMax down_motor, top_motor;
  
  private Shooter() {
    down_motor = new CANSparkMax(Constants.Shooter.id_down_motor, MotorType.kBrushless); 
    top_motor = new CANSparkMax(Constants.Shooter.id_top_motor, MotorType.kBrushless); 
    down_motor.setIdleMode(IdleMode.kBrake); 
    top_motor.setIdleMode(IdleMode.kBrake); 
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
