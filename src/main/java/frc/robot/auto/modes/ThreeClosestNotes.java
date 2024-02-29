package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.auto.AutoTrajectoryReader;
import frc.robot.auto.IAuto;
import frc.robot.commands.CollectNote;
import frc.robot.commands.FollowPath;
import frc.robot.commands.ResetGyroRedAlliance;
import frc.robot.commands.ShootNote;

public class ThreeClosestNotes implements IAuto {
    private final Trajectory mSpk_Center, mCenter_Spk; 
    private final Pose2d mStartingPose; 
    public ThreeClosestNotes () {
        mSpk_Center = AutoTrajectoryReader.generateTrajectoryFromFile("paths/Spk_Center.path", Constants.createTrajConfig(4, 4.2));
        mCenter_Spk = AutoTrajectoryReader.generateTrajectoryFromFile("paths/Center_Spk.path", Constants.createTrajConfig(4, 4.2));
        mStartingPose = new Pose2d(mSpk_Center.getInitialPose().getTranslation(), Rotation2d.fromDegrees(Telemetry.isRedAlliance() ? 180 : 0)); 
    }

    @Override 
    public Command getAutoCommand () {
        return new SequentialCommandGroup( 
            new ShootNote(), 
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new FollowPath(mSpk_Center, Rotation2d.fromDegrees(Telemetry.isRedAlliance() ? 180 : 0)),
                    new FollowPath(mCenter_Spk, Rotation2d.fromDegrees(Telemetry.isRedAlliance() ? 180 : 0))
                ),
                new SequentialCommandGroup(
                    new WaitCommand(0.5), 
                    new CollectNote()
                )
            ),
            new ShootNote(), 
            Telemetry.isRedAlliance() ? new ResetGyroRedAlliance() : new InstantCommand()
        );
    }
    @Override
    public Pose2d getStartingPose () {
        return mStartingPose; 
    }
}
