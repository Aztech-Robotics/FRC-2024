package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.auto.AutoTrajectoryReader;
import frc.robot.auto.IAuto;
import frc.robot.commands.FollowPath;

public class TestPath implements IAuto {
    private final Trajectory mTestPath;
    private final Pose2d mStartingPose;

    public TestPath () {
        mTestPath = AutoTrajectoryReader.generateTrajectoryFromFile("paths/TestPath.path", Constants.createTrajConfig(4, 4.2)); 
        mStartingPose = new Pose2d(mTestPath.getInitialPose().getTranslation(), new Rotation2d()); 
    }

    @Override
    public Command getAutoCommand () {
        return new FollowPath(mTestPath, new Rotation2d()); 
    }
    
    @Override 
    public Pose2d getStartingPose () {
        return mStartingPose; 
    }
}
