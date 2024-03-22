package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.IAuto;
import frc.robot.commands.ShootNote;

public class SideNote implements IAuto{
    private final Pose2d mStartingPose; 
    private final Command mAutoCommand; 

    public SideNote () {
        mStartingPose = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(60)); 
        mAutoCommand = new ShootNote(); 
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
