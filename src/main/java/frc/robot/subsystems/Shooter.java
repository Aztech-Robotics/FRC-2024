package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ControlBoard;
import frc.robot.Telemetry;
import frc.robot.Constants.StatusAction;

public class Shooter extends SubsystemBase {
  private static Shooter mShooter;
  private final CANSparkMax mDownMotor, mTopMotor; 
  private final SparkPIDController mController; 
  private final RelativeEncoder mEncoder; 
  private StatusAction mStatus = StatusAction.Undefined; 
  private boolean isPercentO = false; 

  private PeriodicIO mPeriodicIO = new PeriodicIO(); 
  public enum ShooterControlState { 
    None, 
    VariableVelocity, 
    ConstantVelocity 
  } 
  private ShooterControlState mControlState = ShooterControlState.None; 
  private double mConstantVel = 0.7; 

  private Shooter() { 
    mDownMotor = new CANSparkMax(Constants.Shooter.id_down, MotorType.kBrushless); 
    mTopMotor = new CANSparkMax(Constants.Shooter.id_top, MotorType.kBrushless); 
    mController = mDownMotor.getPIDController(); 
    mEncoder = mDownMotor.getEncoder(); 
    mEncoder.setVelocityConversionFactor(Constants.Shooter.gear_ratio);
    mController.setP(Constants.Shooter.kp, 0); 
    mController.setI(Constants.Shooter.ki, 0);
    mController.setD(Constants.Shooter.kd, 0);
    mController.setFF(Constants.Shooter.kFF, 0);
    mDownMotor.setIdleMode(IdleMode.kBrake); 
    mTopMotor.setIdleMode(IdleMode.kBrake); 
    mTopMotor.follow(mDownMotor);
    outputTelemetry();
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
    mPeriodicIO.meas_vel = mEncoder.getVelocity(); 
  }

  public void writePeriodicOutputs () {
    mController.setReference(mPeriodicIO.des_vel, isPercentO ? ControlType.kDutyCycle : ControlType.kVelocity);  
  }

  @Override
  public void periodic() {
    readPeriodicInputs();
    if (mControlState == ShooterControlState.None) {
      mPeriodicIO.des_vel = 0; 
    } else if (mControlState == ShooterControlState.VariableVelocity) {
      mPeriodicIO.des_vel = ControlBoard.getLeftYC1().getAsDouble(); 
    } else if (mControlState == ShooterControlState.ConstantVelocity) {
      mPeriodicIO.des_vel = mConstantVel; 
      if (Math.abs(mConstantVel - mPeriodicIO.meas_vel) >= 100) mStatus = StatusAction.Done; 
    }
    writePeriodicOutputs(); 
  } 
  
  public void setShooterControlState (ShooterControlState controlState) {
    if (mControlState != controlState) mControlState = controlState; 
    if (mControlState == ShooterControlState.ConstantVelocity && mStatus != StatusAction.InProcess) {
      mStatus = StatusAction.InProcess;
    } else {
      mStatus = StatusAction.Undefined; 
    }
  }

  public void keepCurrentVel () {
    mConstantVel = mPeriodicIO.meas_vel; 
  }

  public void setConstantVel (double constant_vel) {
    mConstantVel = constant_vel; 
  }

  public StatusAction geStatusAction () {
    return mStatus; 
  }

  private void outputTelemetry () {
    Telemetry.mSwerveTab.addDouble("Velocity", () -> mPeriodicIO.meas_vel); 
    Telemetry.mSwerveTab.addString("StatusAction", () -> mStatus.name()); 
  }
}
