package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.IAuto;

public class ThreeClosestNotes implements IAuto {
    public ThreeClosestNotes () {}

    @Override 
    public Command getAutoCommand () {
        return new SequentialCommandGroup();
    }
    @Override
    public Pose2d getStartingPose () {
        return new Pose2d(); 
    }
}
