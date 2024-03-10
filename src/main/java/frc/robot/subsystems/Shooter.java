package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ControlBoard;
import frc.robot.Telemetry;
import frc.robot.Constants.StatusAction;
import frc.robot.subsystems.LEDS.LEDSControlState;

public class Shooter extends SubsystemBase {
  private static Shooter mShooter;
  private final TalonFX mDownMotor, mTopMotor; 
  private StatusAction mStatus = StatusAction.Undefined; 
  private boolean isPercentO = false; 

  private PeriodicIO mPeriodicIO = new PeriodicIO(); 
  public enum ShooterControlState { 
    None, 
    VariableVelocity, 
    ConstantVelocity 
  } 
  private ShooterControlState mControlState = ShooterControlState.None; 
  private double mConstantVel = 0; 

  private LEDS mLEDS = LEDS.getInstance(); 

  private Shooter() { 
    mDownMotor = new TalonFX(Constants.Shooter.id_down); 
    mTopMotor = new TalonFX(Constants.Shooter.id_top); 
    TalonFXConfiguration gral_config = new TalonFXConfiguration(); 
    gral_config.CurrentLimits = new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withSupplyCurrentThreshold(40).withSupplyTimeThreshold(0.1).withSupplyCurrentLimitEnable(true); 
    gral_config.Feedback = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor); 
    gral_config.Slot0 = new Slot0Configs().withKP(Constants.Shooter.kp).withKI(Constants.Shooter.ki).withKD(Constants.Shooter.kd).withKS(Constants.Shooter.ks); 
    gral_config.MotorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake); 
    mDownMotor.getConfigurator().apply(gral_config); 
    mTopMotor.getConfigurator().apply(gral_config);
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
    StatusSignal<Double> velocity = mDownMotor.getVelocity(); 
    velocity.refresh(); 
    mPeriodicIO.meas_vel = velocity.getValue(); 
  }

  public void writePeriodicOutputs () { 
    mDownMotor.setControl(isPercentO ? new DutyCycleOut(mPeriodicIO.des_vel) : new VelocityDutyCycle(mPeriodicIO.des_vel)); 
    mTopMotor.setControl(new Follower(mDownMotor.getDeviceID(), false)); 
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
      if (Math.abs(mConstantVel - mPeriodicIO.meas_vel) <= 100) {
        mStatus = StatusAction.Done; 
        mLEDS.setState(LEDSControlState.SolidGreen);
      } 
    }
    writePeriodicOutputs(); 
  } 
  
  public void setShooterControlState (ShooterControlState controlState) {
    if (mControlState != controlState) mControlState = controlState; 
    if (mControlState == ShooterControlState.ConstantVelocity && mStatus != StatusAction.InProcess) {
      mStatus = StatusAction.InProcess; 
      isPercentO = false; 
    } else {
      mStatus = StatusAction.Undefined; 
      isPercentO = true;
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
    Telemetry.mSwerveTab.addDouble("Velocity", () -> mPeriodicIO.meas_vel).withPosition(8, 3); 
    Telemetry.mDriverTab.addString("StatusAction", () -> mStatus.name()).withPosition(8, 1); 
  }
}
