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
import frc.robot.Constants.Drive;

public class DriveMotionPlanner {    
    private final HolonomicDriveController autoController = new HolonomicDriveController(
        new PIDController(0, 0, 0), new PIDController(0, 0, 0), 
        new ProfiledPIDController(0.0002, 0, 0, new Constraints(Drive.autoLimits.kMaxAngularVelocity, Drive.autoLimits.kMaxAngularAccel))
    );
    private Trajectory currentTrajectory; 
    private Rotation2d targetRotation; 
    private Double startTime;
    private boolean isTrajectoryFinished = false;

    public DriveMotionPlanner () {}

    public void setTrajectory (Trajectory trajectory, Pose2d current_pose, ChassisSpeeds current_speeds, Rotation2d target_rotation){
        currentTrajectory = trajectory; 
        this.targetRotation = target_rotation; 
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
                desired_ChassisSpeeds = autoController.calculate(current_pose, desired_state, targetRotation); 
            } 
        } 
        return desired_ChassisSpeeds; 
    }

    public boolean isTrajectoryFinished() {
        return isTrajectoryFinished;
    }
}
