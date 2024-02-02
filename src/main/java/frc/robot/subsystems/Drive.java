package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerve.DriveMotionPlanner;
import frc.lib.swerve.ModuleState;
import frc.lib.swerve.SwerveDriveKinematics;
import frc.lib.swerve.SwerveDriveOdometry;
import frc.lib.swerve.SwerveModule;
import frc.robot.Constants;
import frc.robot.Constants.SwerveModules;
import frc.robot.ControlBoard;
import frc.robot.Telemetry; 
import frc.robot.Constants.Drive.DriveControlMode;
import frc.robot.Constants.Drive.KinematicLimits;

public class Drive extends SubsystemBase {
  private static Drive mDrive;
  private SwerveModule[] mSwerveModules;
  private Pigeon2 mPigeon = new Pigeon2(Constants.Drive.id_pigeon); 

  private PeriodicIO mPeriodicIO = new PeriodicIO(); 
  public enum DriveControlState {
    TeleopControl,
    HeadingControl, 
    DriveToPose, 
    PathFollowing,
    ForceOrient,
    None
  }
  private DriveControlState mControlState = DriveControlState.None;
  private KinematicLimits mKinematicLimits = Constants.Drive.oneMPSLimits; 
  private SwerveDriveKinematics mSwerveKinematics = new SwerveDriveKinematics(
    new Translation2d(Constants.Drive.wheel_base / 2.0, Constants.Drive.track_width / 2.0),
    new Translation2d(Constants.Drive.wheel_base / 2.0, -Constants.Drive.track_width / 2.0),
    new Translation2d(-Constants.Drive.wheel_base / 2.0, Constants.Drive.track_width / 2.0),
    new Translation2d(-Constants.Drive.wheel_base / 2.0, -Constants.Drive.track_width / 2.0)
  ); 
  private SwerveDriveOdometry mOdometry; 
  private DriveMotionPlanner mMotionPlanner; 
  private final Vision mVision = Vision.getInstance();
  private boolean odometryReset = false; 
  private PIDController snapController = new PIDController(3.5, 0, 0); 
  private Field2d field = new Field2d();
  public boolean isTuningMode = false; 
  public boolean recordingDataLog = false; 

  private Drive() {
    mSwerveModules = new SwerveModule[] {
      new SwerveModule(SwerveModules.MOD0, 0),
      new SwerveModule(SwerveModules.MOD1, 1),
      new SwerveModule(SwerveModules.MOD2, 2),
      new SwerveModule(SwerveModules.MOD3, 3)
    };
    mPigeon.reset(); 
    mOdometry = new SwerveDriveOdometry(mSwerveKinematics, getModulesStates()); 
    mMotionPlanner = new DriveMotionPlanner(); 
    for (SwerveModule module : mSwerveModules){
      module.outputTelemetry(); 
    }
    snapController.enableContinuousInput(-Math.PI, Math.PI);
    snapController.setTolerance(Rotation2d.fromDegrees(2).getRadians());
    outputTelemetry(); 
  }

  public static Drive getInstance () {
    if (mDrive == null) {
      mDrive = new Drive();
    }
    return mDrive;
  }

  public static class PeriodicIO { 
    //Inputs
    double timestamp = 0; 
    Rotation2d yawAngle = new Rotation2d(); 
    ModuleState[] meas_module_states = new ModuleState [] {
      new ModuleState(), 
      new ModuleState(), 
      new ModuleState(), 
      new ModuleState() 
    };
    ChassisSpeeds meas_chassis_speeds = new ChassisSpeeds(); 
    Pose2d robot_pose = new Pose2d(); 
    //Outputs 
    DriveControlMode driveControlMode = DriveControlMode.Velocity; 
    ModuleState[] des_module_states = new ModuleState[] {
      new ModuleState(),
      new ModuleState(),
      new ModuleState(),
      new ModuleState()
    }; 
    ChassisSpeeds des_chassis_speeds = new ChassisSpeeds(); 
    Rotation2d heading_setpoint = new Rotation2d(); 
  }

  public void readPeriodicInputs () {
    mPeriodicIO.timestamp = Timer.getFPGATimestamp(); 
    StatusSignal<Double> yawAngle = mPigeon.getYaw(); 
    yawAngle.refresh();
    mPeriodicIO.yawAngle = Rotation2d.fromDegrees(yawAngle.getValue()); 
    for (SwerveModule module : mSwerveModules) {
      module.readPeriodicInputs();
    }
    mPeriodicIO.meas_module_states = getModulesStates(); 
    mPeriodicIO.meas_chassis_speeds = mSwerveKinematics.toChassisSpeeds(mPeriodicIO.meas_module_states); 
    mOdometry.update(mPeriodicIO.yawAngle, mPeriodicIO.meas_module_states); 
    Optional<Pose2d> visionPose = mVision.getVisionPose(); 
    if (visionPose.isPresent()) mOdometry.addVisionMeasurement(visionPose.get(), mVision.getTimestamp(mPeriodicIO.timestamp)); 
    mPeriodicIO.robot_pose =  mOdometry.getPoseMeters(); 
    field.setRobotPose(mPeriodicIO.robot_pose); 
  }

  public void writePeriodicOutputs () {
    updateSetpoint();
    setModulesStates(mPeriodicIO.des_module_states); 
    for (SwerveModule module : mSwerveModules) {
      module.writePeriodicOutputs(); 
    }
  }
  
  @Override
  public void periodic() {
    readPeriodicInputs();
    if (mControlState == DriveControlState.TeleopControl || mControlState == DriveControlState.HeadingControl) {
      mPeriodicIO.des_chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        ControlBoard.getLeftYC0().getAsDouble() * mKinematicLimits.kMaxDriveVelocity, 
        ControlBoard.getLeftXC0().getAsDouble() * mKinematicLimits.kMaxDriveVelocity, 
        ControlBoard.getRightXC0().getAsDouble() * mKinematicLimits.kMaxAngularVelocity,
        mPeriodicIO.yawAngle
      );
      if (mControlState == DriveControlState.HeadingControl) {
        if (snapController.getSetpoint() != mPeriodicIO.heading_setpoint.getRadians()) {
          snapController.setSetpoint(mPeriodicIO.heading_setpoint.getRadians()); 
          snapController.reset();
        }
        if (Math.abs(mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond) > Math.PI / 2) {
          mControlState = DriveControlState.TeleopControl; 
        } else { 
          mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond = snapController.calculate(mPeriodicIO.yawAngle.getRadians()); 
        }
      }
    } else if (mControlState == DriveControlState.PathFollowing) {
      mPeriodicIO.des_chassis_speeds = mMotionPlanner.update(mPeriodicIO.robot_pose, mPeriodicIO.timestamp);
    } 
    writePeriodicOutputs();
    if (isTuningMode) feedTuningMode(); 
  }

  private void updateSetpoint () {
    Pose2d robot_pose_vel = new Pose2d(
      mPeriodicIO.des_chassis_speeds.vxMetersPerSecond * Constants.kLooperDt, 
      mPeriodicIO.des_chassis_speeds.vyMetersPerSecond * Constants.kLooperDt, 
      Rotation2d.fromRadians(mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond * Constants.kLooperDt)
    );
    Twist2d twist_vel = new Pose2d().log(robot_pose_vel);
    ChassisSpeeds wanted_speeds = new ChassisSpeeds(
      twist_vel.dx / Constants.kLooperDt, 
      twist_vel.dy / Constants.kLooperDt, 
      twist_vel.dtheta / Constants.kLooperDt
    );
    if (mControlState == DriveControlState.TeleopControl || mControlState == DriveControlState.HeadingControl) {
      // Limit rotational velocity
      wanted_speeds.omegaRadiansPerSecond = Math.signum(wanted_speeds.omegaRadiansPerSecond) * Math.min(mKinematicLimits.kMaxAngularVelocity, Math.abs(wanted_speeds.omegaRadiansPerSecond));
      // Limit translational velocity
      double velocity_magnitude = Math.hypot(mPeriodicIO.des_chassis_speeds.vxMetersPerSecond, mPeriodicIO.des_chassis_speeds.vyMetersPerSecond);
      if (velocity_magnitude > mKinematicLimits.kMaxDriveVelocity) {
        wanted_speeds.vxMetersPerSecond = (wanted_speeds.vxMetersPerSecond / velocity_magnitude) * mKinematicLimits.kMaxDriveVelocity;
        wanted_speeds.vyMetersPerSecond = (wanted_speeds.vyMetersPerSecond / velocity_magnitude) * mKinematicLimits.kMaxDriveVelocity;
      }
      ModuleState[] prev_module_states = mPeriodicIO.des_module_states.clone(); // Get last setpoint to get differentials
      ChassisSpeeds prev_chassis_speeds = mSwerveKinematics.toChassisSpeeds(prev_module_states); 
  
      double dx = wanted_speeds.vxMetersPerSecond - prev_chassis_speeds.vxMetersPerSecond;
      double dy = wanted_speeds.vyMetersPerSecond - prev_chassis_speeds.vyMetersPerSecond;
      double domega = wanted_speeds.omegaRadiansPerSecond - prev_chassis_speeds.omegaRadiansPerSecond;
  
      double max_velocity_step = mKinematicLimits.kMaxAccel * Constants.kLooperDt;
      double min_translational_scalar = 1.0;
      if (max_velocity_step < Double.MAX_VALUE * Constants.kLooperDt) {
        // Check X
        double x_norm = Math.abs(dx / max_velocity_step);
        min_translational_scalar = Math.min(min_translational_scalar, x_norm);
        // Check Y
        double y_norm = Math.abs(dy / max_velocity_step);
        min_translational_scalar = Math.min(min_translational_scalar, y_norm);
  
        min_translational_scalar *= max_velocity_step;
      }
  
      double max_omega_step = mKinematicLimits.kMaxAngularAccel * Constants.kLooperDt;
      double min_omega_scalar = 1.0;
      if (max_omega_step < Double.MAX_VALUE * Constants.kLooperDt) {
        double omega_norm = Math.abs(domega / max_omega_step);
        min_omega_scalar = Math.min(min_omega_scalar, omega_norm);
  
        min_omega_scalar *= max_omega_step;
      }
  
      wanted_speeds = new ChassisSpeeds(
        prev_chassis_speeds.vxMetersPerSecond + dx * min_translational_scalar, 
        prev_chassis_speeds.vyMetersPerSecond + dy * min_translational_scalar, 
        prev_chassis_speeds.omegaRadiansPerSecond + domega * min_omega_scalar
      );
       
      ModuleState[] real_module_setpoints = mSwerveKinematics.toModuleStates(wanted_speeds);
      mPeriodicIO.des_module_states = real_module_setpoints;

    } else if (mControlState == DriveControlState.PathFollowing) { 
      mPeriodicIO.des_module_states = mSwerveKinematics.toModuleStates(wanted_speeds); 

    } else if (mControlState == DriveControlState.ForceOrient) {
      mPeriodicIO.des_module_states = new ModuleState [] {
        ModuleState.fromSpeeds(Rotation2d.fromDegrees(-45), 0),
        ModuleState.fromSpeeds(Rotation2d.fromDegrees(45), 0),
        ModuleState.fromSpeeds(Rotation2d.fromDegrees(45), 0),
        ModuleState.fromSpeeds(Rotation2d.fromDegrees(-45), 0)
      };
    }
  }

  public void resetModulesToZero () {
    for (SwerveModule module : mSwerveModules) {
      module.resetModule(); 
    }
  }

  private void setModulesStates (ModuleState[] modulesStates) {
    if (mPeriodicIO.driveControlMode == DriveControlMode.PercentOutput) {
      for (ModuleState modState : modulesStates) {
        modState.speedMetersPerSecond = modState.speedMetersPerSecond / mKinematicLimits.kMaxDriveVelocity; 
      }
    }
    for (int i = 0; i < mSwerveModules.length; i++) {
      mSwerveModules[i].setModuleState(modulesStates[i], mPeriodicIO.driveControlMode);
    }
  }

  private ModuleState[] getModulesStates () {
    ModuleState[] moduleStates = new ModuleState[4]; 
    for (int i = 0; i < 4; i++){
      moduleStates[i] = mSwerveModules[i].getModuleState();
    }
    return moduleStates; 
  }

  public void setDesiredChassisSpeeds (ChassisSpeeds chassisSpeeds) {
    mPeriodicIO.des_chassis_speeds = chassisSpeeds; 
  }

  public void resetGyro () {
    mPigeon.reset(); 
  }

  public void setYawAngle (double angle) {
    mPigeon.setYaw(angle); 
  }

  public Rotation2d getYawAngle () {
    return mPeriodicIO.yawAngle; 
  }

  public void toggleDriveControl (){
    mPeriodicIO.driveControlMode = mPeriodicIO.driveControlMode == DriveControlMode.PercentOutput? 
    DriveControlMode.Velocity : DriveControlMode.PercentOutput; 
  }

  public void setDriveControlState (DriveControlState state) {
    if (mControlState != state) mControlState = state; 
  }

  public void setKinematicsLimits (KinematicLimits limits) {
    mKinematicLimits = limits; 
  }

  public void resetOdometry (Pose2d pose) {
    for (SwerveModule module : mSwerveModules) {
      module.resetModule();
    }
    setYawAngle(pose.getRotation().getDegrees());
    mOdometry.resetPosition(getModulesStates(), pose); 
    odometryReset = true; 
  }

  public boolean isReadyForAuto () {
    return odometryReset; 
  }

  public void setTrajectory (Trajectory trajectory, Rotation2d targetRotation) {
    mMotionPlanner.setTrajectory(trajectory, mPeriodicIO.robot_pose, mPeriodicIO.meas_chassis_speeds, targetRotation); 
    mControlState = DriveControlState.PathFollowing; 
  }

  public boolean isTrajectoryFinished () {
    return mControlState == DriveControlState.PathFollowing && mMotionPlanner.isTrajectoryFinished(); 
  }

  public void setHeadingControl (Rotation2d headingSetpoint) {
    mPeriodicIO.heading_setpoint = headingSetpoint; 
    mControlState = DriveControlState.HeadingControl; 
  }

  public void setTuningMode (boolean active) {
    isTuningMode = active; 
    for (SwerveModule module : mSwerveModules) {
      module.setTuningMode(active); 
    }
  }

  private void feedTuningMode () {
    if (!SmartDashboard.containsKey("Drive Kp")) {
      SmartDashboard.putNumber("Drive Kp", SwerveModules.drive_kP); 
      SmartDashboard.putNumber("Drive Ki", SwerveModules.drive_kI); 
      SmartDashboard.putNumber("Drive Kd", SwerveModules.drive_kD); 
      SmartDashboard.putNumber("Steer Kp", SwerveModules.steer_kP); 
      SmartDashboard.putNumber("Steer Ki", SwerveModules.steer_kI); 
      SmartDashboard.putNumber("Steer Kd", SwerveModules.steer_kD); 
    }

    double drive_kp = SmartDashboard.getNumber("Drive Kp", SwerveModules.drive_kP); 
    double drive_ki = SmartDashboard.getNumber("Drive Ki", SwerveModules.drive_kI); 
    double drive_kd = SmartDashboard.getNumber("Drive Kd", SwerveModules.drive_kD); 
    double steer_kp = SmartDashboard.getNumber("Steer Kp", SwerveModules.steer_kP); 
    double steer_ki = SmartDashboard.getNumber("Steer Ki", SwerveModules.steer_kI); 
    double steer_kd = SmartDashboard.getNumber("Steer Kd", SwerveModules.steer_kD); 

    if (SwerveModules.drive_kP != drive_kp) SwerveModules.drive_kP = drive_kp; 
    if (SwerveModules.drive_kI != drive_ki) SwerveModules.drive_kI = drive_ki; 
    if (SwerveModules.drive_kD != drive_kd) SwerveModules.drive_kD = drive_kd; 
    if (SwerveModules.steer_kP != steer_kp) SwerveModules.steer_kP = steer_kp; 
    if (SwerveModules.steer_kI != steer_ki) SwerveModules.steer_kI = steer_ki; 
    if (SwerveModules.steer_kD != steer_kd) SwerveModules.steer_kD = steer_kd; 
  }

  public void recordDataLog (boolean active) {
    for (SwerveModule swerveModule : mSwerveModules) { 
      swerveModule.recordDataLog(active); 
    }
  }

  private void outputTelemetry (){ 
    Telemetry.mDriverTab.add(field).withPosition(0, 0).withSize(7, 4);
    Telemetry.mDriverTab.addDouble("Robot Pose X", () -> mPeriodicIO.robot_pose.getX()).withPosition(7, 3); 
    Telemetry.mDriverTab.addDouble("Robot Pose Y", () -> mPeriodicIO.robot_pose.getY()).withPosition(8, 3); 
  }
}