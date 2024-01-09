package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.IAuto;
import frc.robot.commands.FollowPath;

public class AutoFromSPK implements IAuto {
    private final PathPlannerPath SPK_TO_NOTE_RIGHT_PATH, 
    NOTE_RIGHT_TO_SPK_PATH, 
    SPK_TO_NOTE_CENTER_PATH, 
    NOTE_CENTER_TO_SPK_PATH;
    private final Pose2d startingPose;

    public AutoFromSPK () {
        SPK_TO_NOTE_RIGHT_PATH = PathPlannerPath.fromPathFile("SPK_TO_NOTE_RIGHT"); 
        NOTE_RIGHT_TO_SPK_PATH = PathPlannerPath.fromPathFile("NOTE_RIGHT_TO_SPK");
        SPK_TO_NOTE_CENTER_PATH = PathPlannerPath.fromPathFile("SPK_TO_NOTE_CENTER"); 
        NOTE_CENTER_TO_SPK_PATH = PathPlannerPath.fromPathFile("NOTE_CENTER_TO_SPK"); 
        startingPose = SPK_TO_NOTE_RIGHT_PATH.getPreviewStartingHolonomicPose(); 
    }

    @Override
    public Command getAutoCommand () {
        return new SequentialCommandGroup(
            new WaitCommand(2),
            new FollowPath(SPK_TO_NOTE_RIGHT_PATH),
            new WaitCommand(1),
            new FollowPath(NOTE_RIGHT_TO_SPK_PATH),
            new WaitCommand(2),
            new FollowPath(SPK_TO_NOTE_CENTER_PATH),
            new WaitCommand(1),
            new FollowPath(NOTE_CENTER_TO_SPK_PATH),
            new WaitCommand(2)
        ); 
    }
    
    @Override 
    public Pose2d getStartingPose () {
        return startingPose; 
    }
}
