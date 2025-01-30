package frc.robot;

public class Utility {
    public static boolean isAngleBetween(double myAngle, double startAngle, double endAngle) {
        myAngle = (myAngle + 360) % 360;
        startAngle = (startAngle + 360) % 360;
        endAngle = (endAngle + 360) % 360;

        if (startAngle < endAngle) {
            return myAngle >= startAngle && myAngle <= endAngle;
        } else { // Handles wraparound case
            return myAngle >= startAngle || myAngle <= endAngle;
        }
    }
    public static double convertTo360Range(double angle) {
        return (angle + 360) % 360;
    }
}
