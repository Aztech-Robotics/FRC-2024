package frc.lib;

public class Conversions {
    private static double mod (double x, double y) {
        return x - Math.floor(x / y) * y;
    }

    public static double signedToUnsignedDeg (double angle) {
        return mod(angle, 360);
    }

    public static double unsignedToSignedDeg (double angle) {
        return mod((angle + 180), 360) - 180; 
    }

    public static double MPSToRPS(double velocity, double circumference, double gearRatio) {
        double wheelRPS = ((velocity) / circumference);
        double falconRPS = wheelRPS * gearRatio;
        return falconRPS;
    }
}
