package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.auto.IAuto;
import frc.robot.auto.modes.TestPath;
import frc.robot.auto.modes.TwoClosestNotes;

public class Telemetry {
    public static ShuffleboardTab mSwerveTab = Shuffleboard.getTab("Swerve"); 
    public static ShuffleboardTab mDriverTab = Shuffleboard.getTab("Driver");
    public enum AutoMode {
        DO_NOTHING,
        TEST_PATH, 
        TWO_CLOSEST_NOTES
    }
    private SendableChooser<Alliance> mAllianceChooser = new SendableChooser<>(); 
    private static boolean isRedAlliance = false; 
    private SendableChooser<AutoMode> mAutoChooser = new SendableChooser<>(); 
    private AutoMode mCachedDesiredMode = AutoMode.DO_NOTHING; 
    private Optional<IAuto> mAutoMode = Optional.empty(); 
    
    public Telemetry () {
        mAllianceChooser.setDefaultOption("BLUE", Alliance.Blue);
        mAllianceChooser.addOption("RED", Alliance.Red);
        mDriverTab.add("ALLIANCE", mAllianceChooser).withPosition(7, 0); 
        mAutoChooser.setDefaultOption("DO NOTHING", AutoMode.DO_NOTHING);
        mAutoChooser.addOption("THREE CLOSEST NOTES", AutoMode.TWO_CLOSEST_NOTES); 
        mAutoChooser.addOption("TEST PATH", AutoMode.TEST_PATH); 
        mDriverTab.add("AUTO MODE", mAutoChooser).withPosition(8, 0).withSize(2, 1); 
    }

    public void updateAutoModeCreator () {
        AutoMode desiredAutoMode = mAutoChooser.getSelected(); 
        if (desiredAutoMode == null) desiredAutoMode = AutoMode.DO_NOTHING; 
        boolean alliance_changed = false; 
        if (mAllianceChooser.getSelected() == Alliance.Red) {
            alliance_changed = isRedAlliance ? false : true; 
            isRedAlliance = true;
        } else if (mAllianceChooser.getSelected() == Alliance.Blue) {
            alliance_changed = isRedAlliance ? true : false; 
            isRedAlliance = false; 
        }
        if (mCachedDesiredMode != desiredAutoMode || alliance_changed) {
            mAutoMode = getAutoMode(desiredAutoMode); 
            DriverStation.reportWarning("Regenerando Auto " + desiredAutoMode.name(), true);
        }
        mCachedDesiredMode = desiredAutoMode; 
    }

    private Optional<IAuto> getAutoMode (AutoMode desiredMode) {
        switch (desiredMode) {
            case DO_NOTHING: 
                return Optional.empty();
            case TEST_PATH: 
                return Optional.of(new TestPath()); 
            case TWO_CLOSEST_NOTES: 
                return Optional.of(new TwoClosestNotes()); 
            default: 
                return Optional.empty(); 
        }
    }

    public Optional<IAuto> getAutoModeSelected () {
        return mAutoMode; 
    }

    public static boolean isRedAlliance () {
        return isRedAlliance; 
    }
}
