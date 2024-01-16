package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class FollowPath extends Command {
  private final Drive mDrive = Drive.getInstance(); 
  private final Trajectory trajectory;
  private final Rotation2d targetRotation; 

  public FollowPath (Trajectory trajectory, Rotation2d targetRotation) {
    this.trajectory = trajectory; 
    this.targetRotation = targetRotation; 
    addRequirements(mDrive); 
  }

  @Override
  public void initialize() {
    if (mDrive.isReadyForAuto()) {
      mDrive.setTrajectory(trajectory, targetRotation); 
    }
  }

  @Override
  public boolean isFinished() {
    return mDrive.isTrajectoryFinished();
  }
}
