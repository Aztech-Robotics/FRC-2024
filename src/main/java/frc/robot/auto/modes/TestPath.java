package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.auto.AutoTrajectoryReader;
import frc.robot.auto.IAuto;
import frc.robot.commands.FollowPath;

public class TestPath implements IAuto {
    private final Trajectory mTestPath; 
    private final Pose2d mStartingPose;

    public TestPath () {
        mTestPath = AutoTrajectoryReader.generateTrajectoryFromFile("paths/TestPath.path", Constants.createTrajConfig(4, 4)); 
        mStartingPose = new Pose2d(mTestPath.getInitialPose().getTranslation(), Rotation2d.fromDegrees(Telemetry.isRedAlliance() ? 180 : 0)); 
    }

    @Override
    public Command getAutoCommand () {
        return new SequentialCommandGroup(
            new FollowPath(mTestPath, Rotation2d.fromDegrees(0))
        ); 
    }
    
    @Override 
    public Pose2d getStartingPose () {
        return mStartingPose; 
    }
}
