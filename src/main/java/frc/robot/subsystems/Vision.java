package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.vision.LimelightHelpers;
import frc.robot.Telemetry;

public class Vision {
  private static Vision mVision;
  public enum VisionMode {
    None,
    LookingForAnyTag,
    LookingForSpecificTag 
  }
  private VisionMode mVisionMode; 
  private int mTagInSearch = 0; 
  private boolean mTagSeen = false; 

  private Vision() {
    mVisionMode = VisionMode.None; 
    Telemetry.mDriverTab.addInteger("Tag ID", () -> (long)LimelightHelpers.getFiducialID("limelight")).withPosition(7, 2); 
    Telemetry.mDriverTab.addDouble("VisionPoseX", () -> LimelightHelpers.getBotPose2d("limelight").getX()).withPosition(8, 2); 
    Telemetry.mDriverTab.addDouble("VisionPoseY", () -> LimelightHelpers.getBotPose2d("limelight").getY()).withPosition(9, 2); 
  }

  public static Vision getInstance () {
    if (mVision == null) mVision = new Vision(); 
    return mVision; 
  }

  public Optional<Pose2d> getVisionPose () { 
    switch (mVisionMode) {
      case None: 
        return Optional.empty();
      case LookingForAnyTag: 
        return LimelightHelpers.getFiducialID("limelight") != -1 ? Optional.of(LimelightHelpers.getBotPose2d("limelight")) : Optional.empty();
      case LookingForSpecificTag: 
        return LimelightHelpers.getFiducialID("limelight") == mTagInSearch ? Optional.of(LimelightHelpers.getBotPose2d("limelight")) : Optional.empty();
      default:
        return Optional.empty();
    }
  }

  public void setVisionMode (VisionMode mode) {
    mVisionMode = mode; 
  }

  public void setTagInSearch (int id) {
    mTagInSearch = id; 
    mTagSeen = false;
  }

  public boolean tagHasBeenSeen () {
    return mTagSeen; 
  }

  public double getTimestamp (double timestamp) {
    return timestamp - (LimelightHelpers.getLatency_Pipeline("limelight")/1000.0) - (LimelightHelpers.getLatency_Capture("limelight")/1000.0); 
  }
}
