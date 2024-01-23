package frc.robot;

public class Field {

    public static int getIdLeftSource () {
        return Telemetry.isRedAlliance() ? 0 : 0; 
    }

    public static int getIdRightSource () {
        return Telemetry.isRedAlliance() ? 0 : 0; 
    }

    public static int getIdAmp () {
        return Telemetry.isRedAlliance() ? 0 : 0; 
    }

    public static int getIdSpeaker () {
        return Telemetry.isRedAlliance() ? 0 : 0; 
    }
}
