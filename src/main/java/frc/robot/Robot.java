package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ModeManager.Mode;
import frc.robot.ModeManager.SubMode;
import frc.robot.auto.IAuto;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drive.DriveControlState;
import frc.robot.subsystems.Intake.IntakeControlState;
import frc.robot.subsystems.Shooter.ShooterControlState;

public class Robot extends TimedRobot {
  private Telemetry mTelemetry; 
  private ModeManager mModeManager;
  private Drive mDrive; 
  private Intake mIntake; 
  private Shooter mShooter; 
  private Optional<IAuto> mAutoMode = Optional.empty(); 
  private Command mAutonomousCommand; 
  private static DataLog log; 

  public static DataLog getLog () {
    if (log == null) log = DataLogManager.getLog(); 
    return log; 
  }

  @Override
  public void robotInit() {
    mTelemetry = new Telemetry(); 
    mModeManager = ModeManager.getInstance(); 
    mDrive = Drive.getInstance(); 
    mIntake = Intake.getInstance(); 
    mShooter = Shooter.getInstance(); 
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() { 
    mDrive.setDriveControlState(DriveControlState.None); 
    mIntake.setIntakeControlState(IntakeControlState.None); 
    mShooter.setShooterControlState(ShooterControlState.None);  
    
  }

  @Override
  public void disabledPeriodic() {
    mTelemetry.updateAutoModeCreator();
    mAutoMode = mTelemetry.getAutoModeSelected(); 
  }

  @Override
  public void autonomousInit() {
    if (mAutoMode.isPresent()) {
      mDrive.setKinematicsLimits(Constants.Drive.uncappedLimits); 
      mDrive.resetOdometry(mAutoMode.get().getStartingPose()); 
      mAutonomousCommand = mAutoMode.get().getAutoCommand(); 
      mAutonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {} 

  @Override
  public void teleopInit() {
    if (mAutonomousCommand != null) {
      mAutonomousCommand.cancel();
    }
    mDrive.setKinematicsLimits(Constants.Drive.oneMPSLimits); 
    mDrive.setDriveControlState(DriveControlState.TeleopControl); 
    mIntake.setIntakeControlState(IntakeControlState.VariableVelocity); 
    mShooter.setShooterControlState(ShooterControlState.VariableVelocity);
  }

  @Override
  public void teleopPeriodic() {
    //Driver
    if (ControlBoard.driver.getRightBumperPressed()) {
      mDrive.setYawAngle(0); 
    }
    if (ControlBoard.driver.getPOV() != -1) { 
      mDrive.setHeadingControl(Rotation2d.fromDegrees(ControlBoard.driver.getPOV())); 
    }
    if (ControlBoard.driver.getAButtonPressed()) {
      mIntake.keepCurrentVel();
    }
    if (ControlBoard.driver.getBButtonPressed()) {
      mIntake.setIntakeControlState(IntakeControlState.ConstantVelocity); 
    } else if (ControlBoard.driver.getBButtonReleased()) {
      mIntake.setIntakeControlState(IntakeControlState.VariableVelocity); 
    } 
    if (ControlBoard.driver.getXButtonPressed()) {
      mShooter.keepCurrentVel();
    }
    if (ControlBoard.driver.getYButtonPressed()) {
      mShooter.setShooterControlState(ShooterControlState.ConstantVelocity); 
    } else if (ControlBoard.driver.getBButtonReleased()) {
      mShooter.setShooterControlState(ShooterControlState.VariableVelocity);  
    } 
    //Operator
    if (ControlBoard.operator.getLeftBumperPressed()) mModeManager.toggleMode(); 

    if (ControlBoard.operator.getAButtonPressed()) {
      mModeManager.setSubMode(mModeManager.getMode() == Mode.PickUp ? SubMode.Floor : SubMode.FromSpeaker);
    }
    if (ControlBoard.operator.getBButtonPressed()) {
      mModeManager.setSubMode(mModeManager.getMode() == Mode.PickUp ? SubMode.Closest_Source : SubMode.FromCenterPodium);
    }
    if (ControlBoard.operator.getXButtonPressed()) {
      mModeManager.setSubMode(mModeManager.getMode() == Mode.PickUp ? SubMode.None : SubMode.Amp); 
    }
    if (ControlBoard.operator.getYButtonPressed()) {
      mModeManager.setSubMode(mModeManager.getMode() == Mode.PickUp ? SubMode.Furthest_Source : SubMode.FromLeftPodium); 
    }
  }

  @Override
  public void testInit() { 
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
