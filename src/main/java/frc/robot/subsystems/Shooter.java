package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase { 
  private static Shooter mShooter; 
  
  private Shooter() {} 

  public static Shooter getInstance () { 
    if (mShooter == null) { 
      mShooter = new Shooter(); 
    }
    return mShooter; 
  }

  @Override
  public void periodic() {}
}
