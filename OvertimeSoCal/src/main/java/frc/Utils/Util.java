package frc.Utils;

public class Util {
    /**
     * @return Returns true when the two given values are within range of the epsilon
     */
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }
}
