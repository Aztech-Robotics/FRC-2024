package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ControlBoard;

public class Shooter extends SubsystemBase {
  private static Shooter mShooter;
  private final CANSparkMax mDownMotor, mTopMotor; 

  private PeriodicIO mPeriodicIO = new PeriodicIO(); 
  public enum ShooterControlState { 
    None, 
    VariableVelocity, 
    ConstantVelocity 
  } 
  private ShooterControlState mControlState = ShooterControlState.None; 
  private double mConstantVel = 0; 

  private Shooter() { 
    mDownMotor = new CANSparkMax(Constants.Intake.id_intake, MotorType.kBrushless); 
    mTopMotor = new CANSparkMax(Constants.Intake.id_rollers, MotorType.kBrushless); 
    mDownMotor.setIdleMode(IdleMode.kBrake); 
    mTopMotor.setIdleMode(IdleMode.kBrake); 
  }

  public static Shooter getInstance () {
    if (mShooter == null) mShooter = new Shooter(); 
    return mShooter; 
  }

  public static class PeriodicIO {
    //Inputs 
    double meas_vel = 0; 
    //Outputs 
    double des_vel = 0; 
  }

  public void readPeriodicInputs () {
    mPeriodicIO.meas_vel = mDownMotor.get(); 
  }

  public void writePeriodicOutputs () {
    mDownMotor.set(mPeriodicIO.des_vel); 
    mTopMotor.set(mPeriodicIO.des_vel);
  }

  @Override
  public void periodic() {
    readPeriodicInputs();
    if (mControlState == ShooterControlState.None) {
      mPeriodicIO.des_vel = 0; 
    } else if (mControlState == ShooterControlState.VariableVelocity) {
      mPeriodicIO.des_vel = ControlBoard.driver.getLeftTriggerAxis(); 
    } else if (mControlState == ShooterControlState.ConstantVelocity) {
      mPeriodicIO.des_vel = mConstantVel; 
    }
    writePeriodicOutputs(); 
  } 

  public void setShooterControlState (ShooterControlState controlState) {
    if (mControlState != controlState) mControlState = controlState; 
  }

  public void keepCurrentVel () {
    mConstantVel = mPeriodicIO.meas_vel; 
  }

  public void setConstantVel (double constant_vel) {
    mConstantVel = constant_vel; 
  }
}
