package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ControlBoard;

public class Intake extends SubsystemBase {
  private static Intake mIntake;
  private final CANSparkMax mIntakeMotor; 

  private PeriodicIO mPeriodicIO = new PeriodicIO(); 
  public enum IntakeControlState { 
    None, 
    VariableVelocity, 
    ConstantVelocity 
  } 
  private IntakeControlState mControlState = IntakeControlState.None; 
  private double mConstantVel = 0; 

  private Intake() { 
    mIntakeMotor = new CANSparkMax(Constants.Intake.id_intake_motor, MotorType.kBrushless); 
    mIntakeMotor.setIdleMode(IdleMode.kBrake); 
  }

  public static Intake getInstance () {
    if (mIntake == null) mIntake = new Intake(); 
    return mIntake; 
  }

  public static class PeriodicIO {
    //Inputs 
    double meas_vel = 0; 
    //Outputs 
    double des_vel = 0; 
  }

  public void readPeriodicInputs () {
    mPeriodicIO.meas_vel = mIntakeMotor.get(); 
  }

  public void writePeriodicOutputs () {
    mIntakeMotor.set(mPeriodicIO.des_vel);
  }

  @Override
  public void periodic() {
    readPeriodicInputs();
    if (mControlState == IntakeControlState.None) {
      mPeriodicIO.des_vel = 0; 
    } else if (mControlState == IntakeControlState.VariableVelocity) {
      mPeriodicIO.des_vel = ControlBoard.getTriggersC0().getAsDouble(); 
    } else if (mControlState == IntakeControlState.ConstantVelocity) {
      mPeriodicIO.des_vel = mConstantVel; 
    }
    writePeriodicOutputs(); 
  } 

  public void setIntakeControlState (IntakeControlState controlState) {
    if (mControlState != controlState) mControlState = controlState; 
  }

  public void keepCurrentVel () {
    mConstantVel = mPeriodicIO.meas_vel; 
  }

  public void setConstantVel (double constant_vel) {
    mConstantVel = constant_vel; 
  }
}
