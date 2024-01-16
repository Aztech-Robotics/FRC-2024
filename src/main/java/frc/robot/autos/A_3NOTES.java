package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathSegment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.AutoTrajectoryReader;
import frc.lib.IAuto;
import frc.lib.WaypointReader;
import frc.robot.Robot;
import frc.robot.commands.FollowPath;

public class A_3NOTES implements IAuto {
    private final Trajectory spk_right;
    private Pose2d startingPose;

    public A_3NOTES () {
        spk_right = AutoTrajectoryReader.generateTrajectoryFromFile(null, null); 
        startingPose = new Pose2d(spk_right.getInitialPose().getTranslation(), new Rotation2d()); 
    }

    public void initializeTrajectory () { 
        
    }

    @Override
    public Command getAutoCommand () {
        return new SequentialCommandGroup( 
        ); 
    }
    
    @Override 
    public Pose2d getStartingPose () {
        if (startingPose == null) startingPose = new Pose2d(); 
        return startingPose; 
    }
}
