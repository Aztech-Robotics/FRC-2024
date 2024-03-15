package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ControlBoard;
import frc.robot.Telemetry;

public class Intake extends SubsystemBase {
  private static Intake mIntake;
  private final CANSparkMax mIntakeMotor, mRollerMotor; 
  private final Rev2mDistanceSensor mSensor; 

  private PeriodicIO mPeriodicIO = new PeriodicIO(); 
  public enum IntakeControlState { 
    None, 
    TakingNote, 
    ReleasingNote,
    VariableVelocity
  } 
  private IntakeControlState mControlState = IntakeControlState.None; 
  private final double kDistanceNoteInside = 10; 
  private final double kDistanceNoteOutside = 12; 
  private double mConstantVel = 0.35; 

  private Intake() { 
    mIntakeMotor = new CANSparkMax(Constants.Intake.id_intake, MotorType.kBrushless); 
    mRollerMotor = new CANSparkMax(Constants.Intake.id_rollers, MotorType.kBrushless);
    mSensor = new Rev2mDistanceSensor(Port.kOnboard); 
    mIntakeMotor.enableVoltageCompensation(12); 
    mRollerMotor.enableVoltageCompensation(12); 
    mIntakeMotor.setSmartCurrentLimit(30); 
    mRollerMotor.setSmartCurrentLimit(30); 
    mIntakeMotor.setIdleMode(IdleMode.kBrake); 
    mRollerMotor.setIdleMode(IdleMode.kBrake); 
    mIntakeMotor.setInverted(false);
    mSensor.setAutomaticMode(true); 
    outputTelemetry();
  }

  public static Intake getInstance () {
    if (mIntake == null) mIntake = new Intake(); 
    return mIntake; 
  }

  public static class PeriodicIO {
    //Inputs 
    Double distance_sensor = 0.0; 
    double vel_roller = 0; 
    //Outputs 
    double des_vel_intake = 0; 
    double des_vel_roller = 0; 
  }

  public void readPeriodicInputs () {
    mPeriodicIO.vel_roller = mRollerMotor.get();
    mPeriodicIO.distance_sensor = mSensor.isRangeValid() ? mSensor.GetRange() : Double.NaN; 
  }

  public void writePeriodicOutputs () {
    mIntakeMotor.set(mPeriodicIO.des_vel_intake); 
    mRollerMotor.set(mPeriodicIO.des_vel_roller);
  }

  @Override
  public void periodic() {
    readPeriodicInputs();
    if (mControlState == IntakeControlState.None) {
      mPeriodicIO.des_vel_intake = 0; 
      mPeriodicIO.des_vel_roller = 0; 
    } else if (mControlState == IntakeControlState.TakingNote) {
      mPeriodicIO.des_vel_intake = mConstantVel * Constants.Intake.ratio_intake_roller;
      mPeriodicIO.des_vel_roller = mConstantVel; 
      if (!mPeriodicIO.distance_sensor.isNaN()) {
        if (mPeriodicIO.distance_sensor <= kDistanceNoteInside) {
          mControlState = IntakeControlState.VariableVelocity; 
        }
      } 
    } else if (mControlState == IntakeControlState.ReleasingNote) {
      mPeriodicIO.des_vel_intake = 0;
      mPeriodicIO.des_vel_roller = mConstantVel; 
      if (!mPeriodicIO.distance_sensor.isNaN()) {
        if (mPeriodicIO.distance_sensor >= kDistanceNoteOutside) {
          mControlState = IntakeControlState.VariableVelocity; 
        }
      } 
    } else if (mControlState == IntakeControlState.VariableVelocity) {
      double output_control = ControlBoard.driver.getRightTriggerAxis() - ControlBoard.driver.getLeftTriggerAxis(); 
      mPeriodicIO.des_vel_intake = output_control * Constants.Intake.ratio_intake_roller; 
      mPeriodicIO.des_vel_roller = output_control;    
    }
    writePeriodicOutputs(); 
  } 

  public void setControlState (IntakeControlState controlState) {
    if (mControlState != controlState) mControlState = controlState; 
  }

  public IntakeControlState getControlState () {
    return mControlState; 
  }

  public void setConstantVel (double constant_vel) {
    mConstantVel = constant_vel; 
  }

  private void outputTelemetry () {
    Telemetry.mSwerveTab.addDouble("PowerIntake", () -> mPeriodicIO.vel_roller).withPosition(8, 2); 
    Telemetry.mDriverTab.addDouble("Distance", () -> mPeriodicIO.distance_sensor).withPosition(7, 1); 
  }
}
