package frc.robot.auto;

import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;
import frc.robot.Telemetry;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;

public class WaypointReader {
    private static final double PATHWEAVER_Y_OFFSET = 8.01367968; 
    /**
     * Get control vector list from path file
     * @param pathName Specify the {THIS} in src/main/deploy/waypoints/{THIS}.path
     * @return control vectors from file
     */
    public static TrajectoryGenerator.ControlVectorList getControlVectors(Path path) throws IOException {

        TrajectoryGenerator.ControlVectorList controlVectors = new TrajectoryGenerator.ControlVectorList();

        try (BufferedReader reader = new BufferedReader(new FileReader(path.toFile()))){
            boolean skippedFirst = false;
            String line = reader.readLine();
            while (line != null) {
                if (!skippedFirst || !line.contains(",")) {
                    skippedFirst = true;
                    line = reader.readLine();
                    continue;
                }
                String[] split = line.split(",");
                double x = Double.parseDouble(split[0]);
                double x_tan = Double.parseDouble(split[2]);
                double y = Double.parseDouble(split[1]) + PATHWEAVER_Y_OFFSET; 
                double y_tan = Double.parseDouble(split[3]);
                if (Telemetry.isRedAlliance()) {
                    x = Constants.Field.length - x;
                    x_tan = - x_tan;
                }
                controlVectors.add(
                    new Spline.ControlVector(
                        new double[]{x, x_tan, 0},
                        new double[]{y, y_tan, 0}
                    )
                );
                line = reader.readLine();
            }
        }
        return controlVectors;
    }
}
