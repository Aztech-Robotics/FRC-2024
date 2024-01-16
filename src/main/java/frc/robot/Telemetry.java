package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.lib.IAuto;
import frc.robot.autos.A_3NOTES;

public class Telemetry {
    public static ShuffleboardTab swerveTab = Shuffleboard.getTab("SwerveData"); 
    public static SendableChooser<IAuto> autoChooser = new SendableChooser<>(); 
    public static SendableChooser<Alliance> allianceChooser = new SendableChooser<>(); 

    public static void displayAutos () {
        allianceChooser.setDefaultOption("Blue", Alliance.Blue);
        allianceChooser.addOption("Red", Alliance.Red);
        swerveTab.add(allianceChooser); 
        A_3NOTES autoTest = new A_3NOTES(); 
        autoChooser.setDefaultOption("None", null);
        autoChooser.addOption("AutoTest", autoTest);
        swerveTab.add(autoChooser); 
    }
}
