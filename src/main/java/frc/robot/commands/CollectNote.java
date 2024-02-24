package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeControlState;

public class CollectNote extends Command {
  private final Intake mIntake = Intake.getInstance(); 
  public CollectNote() {}

  @Override
  public void initialize() {
    mIntake.setControlState(IntakeControlState.TakingNote);
  }

  @Override
  public boolean isFinished() {
    return mIntake.getControlState() == IntakeControlState.VariableVelocity;
  }
}
