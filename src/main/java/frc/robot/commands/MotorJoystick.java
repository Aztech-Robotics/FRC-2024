package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ControlBoard;
import frc.robot.subsystems.Shooter;

public class MotorJoystick extends Command {
  private Shooter mMotor = Shooter.getInstance();
  public MotorJoystick() {
    addRequirements(mMotor);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double value = ControlBoard.driver.getRightBumper() ? 0.65 : 0;
    if (Math.abs(value) < 0.2) value = 0; 
    mMotor.setSpeed(value);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
