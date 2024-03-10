package frc.robot;

import java.util.Optional;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.IAuto;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber.ClimberControlState;
import frc.robot.subsystems.Drive.DriveControlState;
import frc.robot.subsystems.Intake.IntakeControlState;
import frc.robot.subsystems.Shooter.ShooterControlState;

public class Robot extends TimedRobot {
  private Telemetry mTelemetry; 
  private Drive mDrive; 
  private Intake mIntake; 
  private Shooter mShooter; 
  private Climber mClimber; 
  private Optional<IAuto> mAutoMode = Optional.empty(); 
  private Command mAutonomousCommand; 

  @Override
  public void robotInit() {
    mTelemetry = new Telemetry(); 
    mDrive = Drive.getInstance(); 
    mIntake = Intake.getInstance(); 
    mShooter = Shooter.getInstance(); 
    mClimber = Climber.getInstance(); 
    CameraServer.startAutomaticCapture("camera", 0); 
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() { 
    mDrive.setDriveControlState(DriveControlState.None); 
    mIntake.setControlState(IntakeControlState.None);
    mShooter.setShooterControlState(ShooterControlState.None); 
    mClimber.setControlState(ClimberControlState.None);
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
    mIntake.setControlState(IntakeControlState.VariableVelocity);
    mShooter.setShooterControlState(ShooterControlState.VariableVelocity); 
    mClimber.setControlState(ClimberControlState.PercentOutput);
  }

  @Override
  public void teleopPeriodic() {
    //Driver

    if (ControlBoard.driver.getYButtonPressed()) {
      mDrive.setYawAngle(0); 
    }
    if (ControlBoard.driver.getPOV() != -1) { 
      mDrive.setHeadingControl(Rotation2d.fromDegrees(ControlBoard.driver.getPOV())); 
    }

    //Operator
    if (ControlBoard.driver.getAButtonPressed()) {
      mIntake.setControlState(IntakeControlState.TakingNote); 
    } else if (ControlBoard.operator.getBButtonPressed()) { 
      mIntake.setControlState(IntakeControlState.ReleasingNote); 
    } else if (ControlBoard.driver.getXButtonPressed() || ControlBoard.operator.getXButtonPressed()) {
      mIntake.setControlState(IntakeControlState.VariableVelocity);
    }

    if (ControlBoard.operator.getRightBumperPressed()) {
      mShooter.setConstantVel(2800);
      mShooter.setShooterControlState(ShooterControlState.ConstantVelocity); 
    } else if (ControlBoard.operator.getRightBumperReleased()) {
      mShooter.setShooterControlState(ShooterControlState.VariableVelocity);
    }

    if (ControlBoard.operator.getLeftBumperPressed()) {
      mShooter.setConstantVel(1000);
      mShooter.setShooterControlState(ShooterControlState.ConstantVelocity); 
    } else if (ControlBoard.operator.getLeftBumperReleased()) {
      mShooter.setShooterControlState(ShooterControlState.VariableVelocity);
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
