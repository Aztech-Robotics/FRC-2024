package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.auto.AutoTrajectoryReader;
import frc.robot.auto.IAuto;
import frc.robot.commands.FollowPath;
import frc.robot.commands.ResetGyroRedAlliance;

public class ThreeClosestNotes implements IAuto {
    private final Trajectory mSpk_Right, mRight_Spk, mSpk_Center, mCenter_Spk, mSpk_Left, mLeft_Spk; 
    private final Pose2d mStartingPose; 
    public ThreeClosestNotes () {
        mSpk_Right = AutoTrajectoryReader.generateTrajectoryFromFile("paths/Spk_Right.path", Constants.createTrajConfig(4, 4.2));
        mRight_Spk = AutoTrajectoryReader.generateTrajectoryFromFile("paths/Right_Spk.path", Constants.createTrajConfig(4, 4.2));
        mSpk_Center = AutoTrajectoryReader.generateTrajectoryFromFile("paths/Spk_Center.path", Constants.createTrajConfig(4, 4.2));
        mCenter_Spk = AutoTrajectoryReader.generateTrajectoryFromFile("paths/Center_Spk.path", Constants.createTrajConfig(4, 4.2));
        mSpk_Left = AutoTrajectoryReader.generateTrajectoryFromFile("paths/Spk_Left.path", Constants.createTrajConfig(4, 4.2));
        mLeft_Spk = AutoTrajectoryReader.generateTrajectoryFromFile("paths/Left_Spk.path", Constants.createTrajConfig(4, 4.2)); 
        mStartingPose = new Pose2d(mSpk_Right.getInitialPose().getTranslation(), Rotation2d.fromDegrees(Telemetry.isRedAlliance() ? 180 : 0)); 
    }

    @Override 
    public Command getAutoCommand () {
        return new SequentialCommandGroup( 
            new FollowPath(mSpk_Right, Rotation2d.fromDegrees(Telemetry.isRedAlliance() ? 180 : 0)),
            new FollowPath(mRight_Spk, Rotation2d.fromDegrees(Telemetry.isRedAlliance() ? 180 : 0)),
            new FollowPath(mSpk_Center, Rotation2d.fromDegrees(Telemetry.isRedAlliance() ? 180 : 0)),
            new FollowPath(mCenter_Spk, Rotation2d.fromDegrees(Telemetry.isRedAlliance() ? 180 : 0)),
            new FollowPath(mSpk_Left, Rotation2d.fromDegrees(Telemetry.isRedAlliance() ? 180 : 0)),
            new FollowPath(mLeft_Spk, Rotation2d.fromDegrees(Telemetry.isRedAlliance() ? 180 : 0)),
            Telemetry.isRedAlliance() ? new ResetGyroRedAlliance() : new InstantCommand()
        );
    }
    @Override
    public Pose2d getStartingPose () {
        return mStartingPose; 
    }
}
