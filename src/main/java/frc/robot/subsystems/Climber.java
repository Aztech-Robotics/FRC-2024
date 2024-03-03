package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControlBoard;
import frc.robot.Telemetry;

public class Climber extends SubsystemBase {
  private static Climber mClimber; 
  private final CANSparkMax mTelRight, mTelLeft; 
  private final SparkPIDController mRightController; 
  private final RelativeEncoder mRightEncoder;  
  private PeriodicIO mPeriodicIO = new PeriodicIO(); 
  public enum ClimberControlState {
    None, 
    PercentOutput,
    PositionOutput
  }
  private ClimberControlState mControlState = ClimberControlState.None; 
  private double mTargetPosition = 0; 

  private Climber() {
    mTelRight = new CANSparkMax(18, MotorType.kBrushless); 
    mTelLeft = new CANSparkMax(19, MotorType.kBrushless); 
    mRightController = mTelRight.getPIDController(); 
    mRightEncoder = mTelRight.getEncoder(); 
    mTelRight.setIdleMode(IdleMode.kBrake); 
    mTelLeft.setIdleMode(IdleMode.kBrake); 
    mTelLeft.follow(mTelRight); 
    mTelRight.setSoftLimit(SoftLimitDirection.kForward, 0); 
    mTelRight.enableSoftLimit(SoftLimitDirection.kForward, true); 
    mTelRight.setSoftLimit(SoftLimitDirection.kReverse, (float)-1.2); 
    mTelRight.enableSoftLimit(SoftLimitDirection.kReverse, true); 
    outputTelemetry();
  }

  public static Climber getInstance () {
    if (mClimber == null) mClimber = new Climber(); 
    return mClimber; 
  }

  public static class PeriodicIO {
    //Inputs 
    double meas_vel = 0; 
    double meas_pos = 0; 
    //Outputs 
    double des_vel = 0; 
    double des_pos = 0; 
  }

    public void readPeriodicInputs () {
    mPeriodicIO.meas_vel = mTelRight.get(); 
    mPeriodicIO.meas_pos = mRightEncoder.getPosition(); 
  }

  public void writePeriodicOutputs () {
    if (mControlState == ClimberControlState.PositionOutput) {
      mRightController.setReference(mPeriodicIO.des_pos, ControlType.kPosition); 

    } else {
      mRightController.setReference(mPeriodicIO.des_vel, ControlType.kDutyCycle); 
    }
  }

  @Override
  public void periodic() {
    readPeriodicInputs();
    if (mControlState == ClimberControlState.None) {
      mPeriodicIO.des_vel = 0; 
    } else if (mControlState == ClimberControlState.PercentOutput) {
      mPeriodicIO.des_vel = ControlBoard.operator.getRightTriggerAxis() - ControlBoard.operator.getLeftTriggerAxis();
    } else if (mControlState == ClimberControlState.PositionOutput) {
      mPeriodicIO.des_pos = mTargetPosition; 
    }
    writePeriodicOutputs(); 
  }

  public void setControlState (ClimberControlState controlState) {
    if (mControlState != controlState) mControlState = controlState; 
  }

  public ClimberControlState getControlState () {
    return mControlState; 
  } 

  private void outputTelemetry () {
    Telemetry.mDriverTab.addDouble("PosTelRight", () -> mPeriodicIO.meas_pos).withPosition(8, 2); 
  }
}
