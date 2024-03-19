package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ControlBoard;
import frc.robot.Telemetry;

public class Climber extends SubsystemBase {
  private static Climber mClimber; 
  private final CANSparkMax mRightTel, mLeftTel; 
  private final RelativeEncoder mRightEncoder, mLeftEncoder; 
  private final SparkPIDController mRightController, mLeftController; 
  private final SparkLimitSwitch mRightRevLim, mLeftRevLim; 
  private PeriodicIO mPeriodicIO = new PeriodicIO(); 
  public enum ClimberControlState {
    None, 
    PercentOutput,
    PositionOutput
  }
  private ClimberControlState mControlState = ClimberControlState.None; 
  private double mTargetPosition = 0; 

  private Climber() {
    mRightTel = new CANSparkMax(Constants.Climber.id_right_tel, MotorType.kBrushless); 
    mRightTel.setIdleMode(IdleMode.kBrake); 
    mLeftTel = new CANSparkMax(Constants.Climber.id_left_tel, MotorType.kBrushless); 
    mLeftTel.setIdleMode(IdleMode.kBrake); 
    mRightEncoder = mRightTel.getEncoder(); 
    mRightEncoder.setPositionConversionFactor(1/3); 
    mLeftEncoder = mLeftTel.getEncoder(); 
    mLeftEncoder.setPositionConversionFactor(1/3); 
    mRightController = mRightTel.getPIDController(); 
    mRightController.setP(Constants.Climber.kp); 
    mRightController.setI(Constants.Climber.ki); 
    mRightController.setD(Constants.Climber.kd);  
    mRightController.setFF(Constants.Climber.kFF); 
    mLeftController = mLeftTel.getPIDController(); 
    mLeftController.setP(Constants.Climber.kp); 
    mLeftController.setI(Constants.Climber.ki); 
    mLeftController.setD(Constants.Climber.kd);  
    mLeftController.setFF(Constants.Climber.kFF); 
    mRightRevLim = mRightTel.getReverseLimitSwitch(Type.kNormallyOpen); 
    mRightRevLim.enableLimitSwitch(true); 
    mLeftRevLim = mLeftTel.getReverseLimitSwitch(Type.kNormallyOpen); 
    mLeftRevLim.enableLimitSwitch(true); 
    mRightEncoder.setPosition(0); 
    mLeftEncoder.setPosition(0); 
    outputTelemetry();
  }

  public static Climber getInstance () {
    if (mClimber == null) mClimber = new Climber(); 
    return mClimber; 
  }

  public static class PeriodicIO {
    //Inputs 
    double meas_pos = 0; 
    boolean right_limit = false; 
    boolean left_limit = false; 
    //Outputs 
    double des_vel = 0; 
    double des_pos = 0; 
  }

  public void readPeriodicInputs () {
    if (mRightRevLim.isPressed()) mRightEncoder.setPosition(0); 
    if (mLeftRevLim.isPressed()) mLeftEncoder.setPosition(0); 
    mPeriodicIO.meas_pos = mRightEncoder.getPosition(); 

  }
  
  public void writePeriodicOutputs () {
    if (mControlState == ClimberControlState.PositionOutput) {
      mRightController.setReference(mPeriodicIO.des_pos, ControlType.kPosition); 
      mLeftController.setReference(mPeriodicIO.des_pos, ControlType.kPosition); 
    } else {
      mRightController.setReference(mPeriodicIO.des_vel, ControlType.kDutyCycle); 
      mLeftController.setReference(mPeriodicIO.des_vel, ControlType.kDutyCycle); 
    }
  }

  @Override
  public void periodic() {
    readPeriodicInputs();
    if (mControlState == ClimberControlState.None) {
      mPeriodicIO.des_vel = 0; 
    } else if (mControlState == ClimberControlState.PercentOutput) {
      mPeriodicIO.des_vel = ControlBoard.driver.getRightTriggerAxis() - ControlBoard.driver.getLeftTriggerAxis();
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

  public void setTargetPosition (double targetPos) {
    mTargetPosition = targetPos; 
  }

  private void outputTelemetry () {
    Telemetry.mDriverTab.addDouble("PosTelRight", () -> mPeriodicIO.meas_pos).withPosition(8, 2); 
  }
}
