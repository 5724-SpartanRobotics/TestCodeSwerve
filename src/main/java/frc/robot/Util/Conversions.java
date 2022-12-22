package frc.robot.Util;

import frc.robot.Subsystems.Constant.DriveConstants;

public class Conversions {
    public static double twoPi = Math.PI * 2;

    /**
     * Converts falcon motor counts to radians
     * @param counts The counts to be converted to radians
     * @return The radians for the specified counts.
     */
    public static double falconToRadians(double counts){
        return counts * (twoPi / (DriveConstants.turnGearRatio * 2048.0));
    }

    /**
     * Converts radians to Falcon motor counts
     * @param radians the radians to convert to Falcon motor counts
     * @return Falcon Counts - 2048 = 1 rotation of the motor
     */
    public static double radiansToFalcon(double radians){
        return radians / (twoPi / (DriveConstants.turnGearRatio * 2048.0));
    }
}
