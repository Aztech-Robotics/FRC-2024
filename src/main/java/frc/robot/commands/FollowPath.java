package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class FollowPath extends Command {
  private final Drive mDrive = Drive.getInstance(); 
  private PathPlannerPath path;

  public FollowPath (PathPlannerPath path) {
    this.path = path; 
    addRequirements(mDrive);
  }

  @Override
  public void initialize() {
    if (mDrive.isReadyForAuto()) {
      mDrive.setTrajectory(path); 
    }
  }

  @Override
  public boolean isFinished() {
    return mDrive.isTrajectoryFinished();
  }
}
