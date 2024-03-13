package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;

public class LEDS extends SubsystemBase {
  private static LEDS mLEDS; 
  private final DigitalOutput mRio0, mRio1; 
  public enum LEDSControlState {
    SolidAlliance,
    BlinkGreen,
    SolidGreen
  }
  private LEDSControlState mControlState = LEDSControlState.SolidAlliance; 
  private PeriodicIO mPeriodicIO = new PeriodicIO(); 

  public static LEDS getInstance () {
    if (mLEDS == null) mLEDS = new LEDS(); 
    return mLEDS; 
  }

  private LEDS() {
    mRio0 = new DigitalOutput(2); 
    mRio1 = new DigitalOutput(3); 
  }

  public static class PeriodicIO {
    boolean rio0 = false; 
    boolean rio1 = false; 
  }

  private void writePeriodicOutputs () {
    mRio0.set(mPeriodicIO.rio0); 
    mRio1.set(mPeriodicIO.rio1);
  }

  @Override
  public void periodic() {
    switch (mControlState) {
      case SolidAlliance: 
        if (Telemetry.isRedAlliance()) {
          mPeriodicIO.rio0 = false; 
          mPeriodicIO.rio1 = true; 
        } else {
          mPeriodicIO.rio0 = false; 
          mPeriodicIO.rio1 = false; 
        }
      break; 
      case SolidGreen: 
        mPeriodicIO.rio0 = true; 
        mPeriodicIO.rio1 = false; 
      break; 
      case BlinkGreen: 
        mPeriodicIO.rio0 = true; 
        mPeriodicIO.rio1 = true; 
      break; 
    }
    writePeriodicOutputs();
  }

  public void setState (LEDSControlState state) {
    if (mControlState != state) mControlState = state; 
  }
}
