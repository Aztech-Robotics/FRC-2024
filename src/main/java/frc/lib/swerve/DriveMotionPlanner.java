package frc.lib.swerve;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;

public class DriveMotionPlanner {    
    private final HolonomicDriveController driveController = new HolonomicDriveController(
        new PIDController(0, 0, 0), new PIDController(0, 0, 0), 
        new ProfiledPIDController(0, 0, 0, new Constraints(0, 0))
    );
    private Trajectory currentTrajectory; 
    private Rotation2d heading_setpoint; 
    private Double startTime;
    private boolean isTrajectoryFinished = false;

    public DriveMotionPlanner () {}

    public void setTrajectory (Trajectory trajectory, Pose2d current_pose, ChassisSpeeds current_speeds, Rotation2d heading_setpoint){
        currentTrajectory = trajectory; 
        this.heading_setpoint = heading_setpoint; 
        isTrajectoryFinished = false; 
        startTime = Double.NaN; 
    }

    public ChassisSpeeds update (Pose2d current_pose, double current_time){
        ChassisSpeeds desired_ChassisSpeeds = new ChassisSpeeds(); 
        if (currentTrajectory != null){ 
            if (startTime.isNaN()){     
                startTime = Timer.getFPGATimestamp();
            }
            double seconds = current_time - startTime; 
            Trajectory.State desired_state = null; 
            if (seconds <= currentTrajectory.getTotalTimeSeconds()){
                desired_state = currentTrajectory.sample(seconds);
            } else {
                isTrajectoryFinished = true; 
                currentTrajectory = null; 
            }
            if (desired_state != null) {
                desired_ChassisSpeeds = driveController.calculate(current_pose, desired_state, heading_setpoint); 
            } 
        } 
        return desired_ChassisSpeeds; 
    }

    public ChassisSpeeds update (Pose2d current_pose, double current_time, Rotation2d tagAngleOverride){
        ChassisSpeeds desired_ChassisSpeeds = new ChassisSpeeds(); 
        if (currentTrajectory != null){ 
            if (startTime.isNaN()){     
                startTime = Timer.getFPGATimestamp();
            }
            double seconds = current_time - startTime; 
            Trajectory.State desired_state = null; 
            if (seconds <= currentTrajectory.getTotalTimeSeconds()){
                desired_state = currentTrajectory.sample(seconds);
            } else {
                isTrajectoryFinished = true; 
                currentTrajectory = null; 
            }
            if (desired_state != null) {
                desired_ChassisSpeeds = driveController.calculate(current_pose, desired_state, tagAngleOverride); 
            } 
        } 
        return desired_ChassisSpeeds; 
    }

    public boolean isTrajectoryFinished() {
        return isTrajectoryFinished;
    }
}
