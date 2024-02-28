package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.IAuto;
import frc.robot.commands.CollectNote;
import frc.robot.commands.ShootNote;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drive.DriveControlState;
import frc.robot.subsystems.Intake.IntakeControlState;
import frc.robot.subsystems.Shooter.ShooterControlState;

public class Robot extends TimedRobot {
  private Telemetry mTelemetry; 
  private Drive mDrive; 
  private Intake mIntake; 
  private Shooter mShooter; 
  private Optional<IAuto> mAutoMode = Optional.empty(); 
  private Command mAutonomousCommand; 
  private CollectNote mCollectNote; 
  private ShootNote mShootNote; 

  @Override
  public void robotInit() {
    mTelemetry = new Telemetry(); 
    mDrive = Drive.getInstance(); 
    mIntake = Intake.getInstance(); 
    mShooter = Shooter.getInstance(); 
    mCollectNote = new CollectNote(); 
    mShootNote = new ShootNote(); 
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
  }

  @Override
  public void teleopPeriodic() {
    //Driver
    if (ControlBoard.driver.getXButtonPressed()) {
      mDrive.setYawAngle(0); 
    }
    if (ControlBoard.driver.getPOV() != -1) { 
      mDrive.setHeadingControl(Rotation2d.fromDegrees(ControlBoard.driver.getPOV())); 
    }

    //Operator
    //Intake 
    if (ControlBoard.operator.getAButtonPressed()) {
      mIntake.setControlState(IntakeControlState.TakingNote); 
    } else if (ControlBoard.operator.getBButtonPressed()) { 
      mIntake.setControlState(IntakeControlState.ReleasingNote); 
    } else if (ControlBoard.operator.getXButtonPressed()) {
      mIntake.setControlState(IntakeControlState.VariableVelocity);
    }
    //Shooter
    if (ControlBoard.operator.getRightBumperPressed()) {
      mShooter.setShooterControlState(ShooterControlState.ConstantVelocity); 
    } else if (ControlBoard.operator.getRightBumperReleased()) {
      mShooter.setShooterControlState(ShooterControlState.VariableVelocity);
    }
    if (ControlBoard.operator.getYButtonPressed()) {
      mShootNote.schedule(); 
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
