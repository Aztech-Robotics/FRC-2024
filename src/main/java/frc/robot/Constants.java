package frc.robot;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import frc.lib.swerve.SwerveModule.SwerveModuleConstants;

public final class Constants {

    public static double kLooperDt = 0.02; 

    public static class Field {
        public static final double length = 16.54; 
        public static final double width = 8.21; 
    }

    public enum StatusAction {
        InProcess,
        Done,
        Undefined
    }

    public static class Climber {
        public static final int id_right_tel = 18; 
        public static final int id_left_tel = 19; 

        public static final double kp = 0.1; 
        public static final double ki = 0; 
        public static final double kd = 0; 
        public static final double kFF = 0; 
    }

    public static class Shooter {
        public static final int id_top = 16; 
        public static final int id_down = 17; 

        public static final double kp = 0.025; 
        public static final double ki = 0; 
        public static final double kd = 0; 
        public static final double ks = 0; 

        public static final double gear_ratio = ((30/14)*(18/36)); 
    }

    public static class Intake {
        public static final int id_intake = 14; 
        public static final int id_rollers = 15; 
    }

    public static TrajectoryConfig createTrajConfig (double maxVel, double maxAccel) {
        TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAccel);
        config.setStartVelocity(0); 
        config.setEndVelocity(0); 
        return new TrajectoryConfig(maxVel, maxAccel);
    }

    public static class Drive {
        public static final int id_pigeon = 13;
        public static final double track_width = Units.inchesToMeters(19.5); //19.5in 28
        public static final double wheel_base = Units.inchesToMeters(21.5); //21.5in 31.5

        public enum DriveControlMode {
            Velocity,
            PercentOutput
        }

        public static final double maxVelocity = 4.5; //Empirical Max Velocity
        public static final double maxAngularVelocity = 8.9; //Theoretical Max Angular Velocity

        public static class KinematicLimits {
            public double kMaxDriveVelocity = maxVelocity; 
            public double kMaxAccel = Double.MAX_VALUE; 
            public double kMaxAngularVelocity = maxAngularVelocity; 
            public double kMaxAngularAccel = Double.MAX_VALUE; 
            KinematicLimits () {}
            KinematicLimits (double kMaxDriveVelocity, double kMaxAngularVelocity) { 
              this.kMaxDriveVelocity = kMaxDriveVelocity; 
              this.kMaxAngularVelocity = kMaxAngularVelocity; 
            }
            KinematicLimits (double kMaxDriveVelocity, double kMaxAccel, double kMaxAngularVelocity, double kMaxAngularAccel){
              this.kMaxDriveVelocity = kMaxDriveVelocity; 
              this.kMaxAccel = kMaxAccel;
              this.kMaxAngularVelocity = kMaxAngularVelocity; 
              this.kMaxAngularAccel = kMaxAngularAccel;
            }
        } 
        
        public static final KinematicLimits uncappedLimits = new KinematicLimits();
        public static final KinematicLimits autoLimits = new KinematicLimits(4, 4.2, Math.PI*2, Math.PI*2);  
        public static final KinematicLimits oneMPSLimits = new KinematicLimits(4.5, 2*Math.PI); 
    }

    public static class SwerveModules {
        public static final double steering_gear_ratio = 12.8; 
        public static final double drive_gear_ratio = 6.75;
        public static final double wheelCircumference = Math.PI * Units.inchesToMeters(4);

        public static final SwerveModuleConstants MOD0 = new SwerveModuleConstants(1, 2, 3, 0.094482421875);
        public static final SwerveModuleConstants MOD1 = new SwerveModuleConstants(4, 5, 6, 0.123779296875);
        public static final SwerveModuleConstants MOD2 = new SwerveModuleConstants(7, 8, 9, 0.2392578125); 
        public static final SwerveModuleConstants MOD3 = new SwerveModuleConstants(10, 11, 12, -0.002197265625); 

        public static double steer_kP = 2;
        public static double steer_kI = 0; 
        public static double steer_kD = 0;
        public static double steer_kS = 0; 
        
        public static double drive_kP = 0.00019; 
        public static double drive_kI = 0; 
        public static double drive_kD = 0; 
        public static double drive_kS = 0;

    }
}
