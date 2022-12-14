package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.SwerveModule;
import frc.robot.Subsystems.Constant.DriveConstants;
import java.util.*;
import edu.wpi.first.wpilibj.ADIS16448_IMU;

public class DriveTrainSubsystem extends SubsystemBase {
    
    // Swerve modules
    private SwerveModule LF;
    private SwerveModule RF;
    private SwerveModule LB;
    private SwerveModule RB;

    // Gyro for now
    private ADIS16448_IMU gyro = new ADIS16448_IMU();

    // Commands for swerve motion
    private double xvel = 0;
    private double yvel = 0;
    private double turnvel = 0;

    // for calculations, r is center to wheel, L is wheel to wheel
    private double r = 0.51;
    private double L = 0.71;

    public DriveTrainSubsystem() {
        LF = new SwerveModule(DriveConstants.LFTurnMotor, DriveConstants.LFDriveMotor, DriveConstants.LFCanID, DriveConstants.LFOff);
        RF = new SwerveModule(DriveConstants.RFTurnMotor, DriveConstants.RFDriveMotor, DriveConstants.RFCanID, DriveConstants.RFOff);
        LB = new SwerveModule(DriveConstants.LBTurnMotor, DriveConstants.LBDriveMotor, DriveConstants.LBCanID, DriveConstants.LBOff);
        RB = new SwerveModule(DriveConstants.RBTurnMotor, DriveConstants.RBDriveMotor, DriveConstants.RBCanID, DriveConstants.RBOff);
    }

    @Override
    public void periodic() {
        // x and y turn components, temporary for now.
        double xturn = turnvel * 1.4 / 2;
        double yturn = turnvel * 1.4 / 2;

        // TODO Auto-generated method stub
        LF.setCartesian(xvel + xturn, yvel + yturn);
        RF.setCartesian(xvel + yturn, yvel - xturn);
        LB.setCartesian(xvel - yturn, yvel + xturn);
        RB.setCartesian(xvel - xturn, yvel - yturn);
        super.periodic();

        LF.update();
        RF.update();
        LB.update();
        RB.update();
    }

    public void driveCommand(double x, double y, double turn) {
        // Convert to polar
        double ang = Math.atan2(y, x);
        double speed = Math.sqrt(y * y + x * x);
        
        // Increase ang by gyro
        ang += gyro.getAngle();

        // Go back to cartesian
        xvel = Math.cos(ang) * speed;
        yvel = Math.sin(ang) * speed;
        turnvel = turn * 0;
    }
}
