package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.StatusAction;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeControlState;
import frc.robot.subsystems.Shooter.ShooterControlState;

public class ShootNote extends Command {
  private final Intake mIntake = Intake.getInstance(); 
  private final Shooter mShooter = Shooter.getInstance(); 
  private Double mStartTime; 
  private boolean mFlag;
  public ShootNote() {
    addRequirements(mIntake, mShooter);
  }

  @Override
  public void initialize() {
    mStartTime = Double.NaN;
    mFlag = false;  
    mShooter.setShooterControlState(ShooterControlState.ConstantVelocity); 
  }

  @Override
  public void execute() {
    if (mShooter.geStatusAction() == StatusAction.Done) {
      mIntake.setControlState(IntakeControlState.ReleasingNote); 
      mStartTime = Timer.getFPGATimestamp(); 
    }
    if (!mStartTime.isNaN()) {
      if (Timer.getFPGATimestamp() - mStartTime >= 1) {
        mFlag = true; 
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    mShooter.setShooterControlState(ShooterControlState.VariableVelocity);
  }

  @Override
  public boolean isFinished() {
    return mFlag;
  }
}
