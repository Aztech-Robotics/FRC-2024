package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.auto.AutoTrajectoryReader;
import frc.robot.auto.IAuto;
import frc.robot.commands.FollowPath;
import frc.robot.commands.IntakeVel;

public class ThreeClosestNotes implements IAuto {
    private final Trajectory mSpk_Right, mRight_Center, mCenter_Left; 
    private final Pose2d mStartingPose; 
    public ThreeClosestNotes () {
        mSpk_Right = AutoTrajectoryReader.generateTrajectoryFromFile("paths/Spk_Right.path", Constants.createTrajConfig(4, 4)); 
        mRight_Center = AutoTrajectoryReader.generateTrajectoryFromFile("paths/Right_Center.path", Constants.createTrajConfig(4, 4)); 
        mCenter_Left = AutoTrajectoryReader.generateTrajectoryFromFile("paths/Center_Left.path", Constants.createTrajConfig(4, 4)); 
        mStartingPose = new Pose2d(mSpk_Right.getInitialPose().getTranslation(), Rotation2d.fromDegrees(Telemetry.isRedAlliance() ? 180 : 0)); 
    }

    @Override 
    public Command getAutoCommand () {
        return new SequentialCommandGroup( 
            new ParallelRaceGroup(
                new FollowPath(mSpk_Right, Rotation2d.fromDegrees(Telemetry.isRedAlliance() ? 180 : 0)),
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new IntakeVel()
                )
            ),
            new WaitCommand(1),
            new ParallelRaceGroup(
                new FollowPath(mRight_Center, Rotation2d.fromDegrees(Telemetry.isRedAlliance() ? 180 : 0)),
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new IntakeVel()
                )
            ), 
            new WaitCommand(1),
            new ParallelRaceGroup(
                new FollowPath(mCenter_Left, Rotation2d.fromDegrees(Telemetry.isRedAlliance() ? 180 : 0)),
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new IntakeVel()
                )
            )
        );
    }
    @Override
    public Pose2d getStartingPose () {
        return mStartingPose; 
    }
}
