package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Field {

    public static int getIdClosestSource () {
        return Telemetry.isRedAlliance() ? 10 : 1; 
    }

    public static int getIdFurthestSource () {
        return Telemetry.isRedAlliance() ? 9 : 2; 
    }

    public static int getIdAmp () {
        return Telemetry.isRedAlliance() ? 5 : 6; 
    }

    public static int getIdSpeaker () {
        return Telemetry.isRedAlliance() ? 4 : 7; 
    }

    public static Pose2d getPoseInClosestSource () {
        return Telemetry.isRedAlliance() ? new Pose2d(0, 0, Rotation2d.fromDegrees(0)): new Pose2d(0, 0, Rotation2d.fromDegrees(0)); 
    }

    public static Pose2d getPoseInFurthestSource () {
        return Telemetry.isRedAlliance() ? new Pose2d(0, 0, Rotation2d.fromDegrees(0)): new Pose2d(0, 0, Rotation2d.fromDegrees(0)); 
    }

    public static Pose2d getPoseInAmp () {
        return Telemetry.isRedAlliance() ? new Pose2d(0, 0, Rotation2d.fromDegrees(0)): new Pose2d(0, 0, Rotation2d.fromDegrees(0)); 
    }

    public static Pose2d getPoseInSpeaker () {
        return Telemetry.isRedAlliance() ? new Pose2d(0, 0, Rotation2d.fromDegrees(0)): new Pose2d(0, 0, Rotation2d.fromDegrees(0)); 
    }

    public static Pose2d getPoseInCenterPodium () {
        return Telemetry.isRedAlliance() ? new Pose2d(0, 0, Rotation2d.fromDegrees(0)): new Pose2d(0, 0, Rotation2d.fromDegrees(0)); 
    }

    public static Pose2d getPoseInLeftPodium () {
        return Telemetry.isRedAlliance() ? new Pose2d(0, 0, Rotation2d.fromDegrees(0)): new Pose2d(0, 0, Rotation2d.fromDegrees(0)); 
    }
}
