package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeControlState;

public class IntakeVel extends Command {
  private final Intake mIntake = Intake.getInstance(); 
  public IntakeVel() {}

  @Override
  public void initialize() {
    mIntake.setConstantVel(-0.5); 
    mIntake.setIntakeControlState(IntakeControlState.ConstantVelocity); 
  }

  @Override
  public void end(boolean interrupted) {
    mIntake.setIntakeControlState(IntakeControlState.None);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
