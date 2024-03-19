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
import frc.robot.commands.ShootNote;

public class Note_OutLine_Right implements IAuto{
    private final Trajectory mOutLine_Right; 
    private final Pose2d mStartingPose; 
    private final Command mAutoCommand; 

    public Note_OutLine_Right () {
        mOutLine_Right = AutoTrajectoryReader.generateTrajectoryFromFile("paths/OutLine_Right.path", Constants.createTrajConfig(4, 4)); 
        Rotation2d targetRot = Telemetry.isRedAlliance() ? Rotation2d.fromDegrees(-60) : Rotation2d.fromDegrees(-60); 
        mStartingPose = new Pose2d(mOutLine_Right.getInitialPose().getTranslation(), targetRot); 
        mAutoCommand = new SequentialCommandGroup(
            new ShootNote(),
            new FollowPath(mOutLine_Right, targetRot)
        );
    }
    @Override 
    public Command getAutoCommand () {
        return mAutoCommand; 
    }
    @Override
    public Pose2d getStartingPose () {
        return mStartingPose; 
    }
}
