package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class ResetGyroRedAlliance extends Command {
  private final Drive mDrive = Drive.getInstance(); 
  public ResetGyroRedAlliance() {
    addRequirements(mDrive);
  }

  @Override
  public void initialize() {
    Rotation2d new_angle = mDrive.getYawAngle().rotateBy(Rotation2d.fromDegrees(180)); 
    mDrive.setYawAngle(new_angle.getDegrees());
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
