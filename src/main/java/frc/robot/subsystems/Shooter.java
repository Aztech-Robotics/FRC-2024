package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private static Shooter mMotor; 
  private final TalonFX motor1, motor2; 
  private Shooter() {
    motor1 = new TalonFX(14); 
    motor2 = new TalonFX(15); 
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
    motor1.getConfigurator().apply(config); 
    motor2.getConfigurator().apply(config); 
  }

  public static Shooter getInstance () {
    if (mMotor == null) {
      mMotor = new Shooter();
    }
    return mMotor; 
  }

  @Override
  public void periodic() {
  }

  public void setSpeed (double output) {
    motor1.setControl(new DutyCycleOut(output)); 
    motor2.setControl(new Follower(motor1.getDeviceID(), true)); 
    SmartDashboard.putNumber("Output", motor1.get());
  }
}
