package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.VisionMode;

public class ModeManager {
    private static ModeManager mModeManager;
    public enum Mode {
        PickUp,
        Score
    }
    public enum SubMode {
        None,
        //PickUp
        Floor,
        Closest_Source,
        Furthest_Source,
        //Score 
        FromSpeaker, 
        FromCenterPodium, 
        FromLeftPodium,
        Amp 
    }
    public enum Score {
        None,
    }
    private Mode mMode; 
    private SubMode mSubMode; 

    private final Vision mVision = Vision.getInstance();

    private ModeManager () {
        mMode = Mode.PickUp; 
        mSubMode = SubMode.None; 
        Telemetry.mDriverTab.addString("Mode", () -> getMode().name()).withPosition(7, 1);
        Telemetry.mDriverTab.addString("SubMode", () -> getSubMode().name()).withPosition(8, 1);  
    }

    public static ModeManager getInstance () {
        if (mModeManager == null) mModeManager = new ModeManager(); 
        return mModeManager; 
    }

    public Mode getMode () {
        return mMode; 
    }

    public SubMode getSubMode () {
        return mSubMode; 
    }

    public void toggleMode () {
        if (mMode == Mode.PickUp) {
            mMode = Mode.Score; 
            DriverStation.reportWarning("Mode Score Active", false); 
        } else if (mMode == Mode.Score) {
            mMode = Mode.PickUp; 
            DriverStation.reportWarning("Mode PickUp Active", false);
        }
        setSubMode(SubMode.None); 
    }

    public void setSubMode (SubMode subMode) { 
        mSubMode = subMode; 
        updateSubMode();
    }

    public void updateSubMode () {
        switch (mSubMode) {
            case None: 
                mVision.setVisionMode(VisionMode.None); 
                DriverStation.reportWarning("SubMode None", false); 
                return;
            case Floor: 
                mVision.setVisionMode(VisionMode.LookingForAnyTag); 
                DriverStation.reportWarning("SubMode Floor", false); 
                return; 
            case Closest_Source: 
                mVision.setTagInSearch(Field.getIdClosestSource()); 
                DriverStation.reportWarning("SubMode Closest Source", false); 
            break; 
            case Furthest_Source: 
                mVision.setTagInSearch(Field.getIdFurthestSource()); 
                DriverStation.reportWarning("SubMode Furthest Source", false); 
            break; 
            case FromSpeaker: 
                mVision.setTagInSearch(Field.getIdSpeaker()); 
                DriverStation.reportWarning("SubMode From Speaker", false); 
                break;
                case FromCenterPodium:
                mVision.setTagInSearch(Field.getIdSpeaker()); 
                DriverStation.reportWarning("SubMode From Center Podium", false); 
            break; 
            case FromLeftPodium: 
                mVision.setTagInSearch(Field.getIdSpeaker()); 
                DriverStation.reportWarning("SubMode Left Podium", false); 
            break; 
            case Amp:
                mVision.setTagInSearch(Field.getIdAmp()); 
                DriverStation.reportWarning("SubMode From Amp", false); 
            break; 
        }
        mVision.setVisionMode(VisionMode.LookingForSpecificTag); 
    }
}
