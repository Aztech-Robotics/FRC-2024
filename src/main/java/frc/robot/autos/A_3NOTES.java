package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.IAuto;
import frc.robot.Robot;
import frc.robot.commands.FollowPath;

public class A_3NOTES implements IAuto {
    private PathPlannerPath spk_right = PathPlannerPath.fromPathFile("SPK_RIGHT");
    private final Pose2d startingPose;

    public A_3NOTES () {
        if (Robot.flipAlliance()) { 
            spk_right = spk_right.flipPath(); 
        } 
        startingPose = spk_right.getPreviewStartingHolonomicPose(); 
    }

    @Override
    public Command getAutoCommand () {
        return new SequentialCommandGroup( 
            new FollowPath(spk_right) 
        ); 
    }
    
    @Override 
    public Pose2d getStartingPose () {
        return startingPose; 
    }
}
