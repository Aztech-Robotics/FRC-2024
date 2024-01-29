package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.IAuto;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drive.DriveControlState;
import frc.robot.subsystems.Intake.IntakeControlState;

public class Robot extends TimedRobot {
  private Telemetry mTelemetry; 
  private Drive mDrive; 
  private Intake mIntake; 
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
    mDrive = Drive.getInstance(); 
    mIntake = Intake.getInstance(); 
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    mDrive.setDriveControlState(DriveControlState.None); 
    if (mDrive.recordingDataLog) mDrive.recordDataLog(false); 
    if (mDrive.isTuningMode) mDrive.setTuningMode(false);
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
  }

  @Override
  public void teleopPeriodic() {
    if (ControlBoard.driver.getAButtonPressed()) {
      mDrive.setYawAngle(0); 
    }
    if (ControlBoard.driver.getPOV() != -1) {
      mDrive.setHeadingControl(Rotation2d.fromDegrees(ControlBoard.driver.getPOV())); 
    }
    if (ControlBoard.driver.getRightBumperPressed()) {
      mIntake.keepCurrentVel();
    }
    if (ControlBoard.driver.getLeftBumperPressed()) {
      mIntake.setIntakeControlState(IntakeControlState.ConstantVelocity); 
    } else if (ControlBoard.driver.getLeftBumperReleased()) {
      mIntake.setIntakeControlState(IntakeControlState.VariableVelocity); 
    }
  }

  @Override
  public void testInit() { 
    mDrive.setKinematicsLimits(Constants.Drive.oneMPSLimits); 
    mDrive.setDriveControlState(DriveControlState.TeleopControl); 
    mDrive.setTuningMode(true); 
    mDrive.recordDataLog(true);
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
