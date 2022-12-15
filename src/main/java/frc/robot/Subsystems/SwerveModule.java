package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.print.CancelablePrintJob;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import org.opencv.core.Mat;

public class SwerveModule {

    // Offset
    private double offset = 0 ;

    // Decleration of swerve components
    private WPI_TalonFX turn;
    private WPI_TalonFX drive;
    private CANCoder canCoder;
    private CANCoderConfiguration config;

    // Constants for angle setting of swerve
    private double m = -5;
    private double b = 4;
    private double maxturn = 0.5;
    private double dif = 0;

    // Desired speed and angle values for swerve
    private double driveSpeed = 0;
    private double driveAngle = 0;

    // Limit for maximum drive speed
    private double maxdrive = 0.3;

    // isInverted is used when the wheel is facing the 
    // opposite direction and should run backwards instead
    private double isInverted = 1;

    // This value is used for whether or not the optimal path
    // to the desired angle loops around, or goes through 0
    private boolean loopAround = false;

    // Requested angle from main
    private double requestedAngle = 0;

    private double canCoderModified = 0;

    private PIDController turnPID;
    private int turnID = 0;

    // Takes ID's of swerve components when called.
    public SwerveModule(int turnMotor, int driveMotor, int canID, double off) {

        // Set the offset
        offset = off;
        turnID = turnMotor;

        // IDs for cancoder and falcons.
        turn = new WPI_TalonFX(turnMotor);
        drive = new WPI_TalonFX(driveMotor);
        canCoder = new CANCoder(canID);
        turn.configFactoryDefault();
        turn.setNeutralMode(NeutralMode.Brake);
        turn.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        turn.config_kP(0, 0.1);
        turn.config_kI(0, 0);
        turn.config_kD(0, 0);
        turn.configForwardSoftLimitEnable(false);
        turn.set(ControlMode.Position, 0);

        // cancoder settings.
        config = new CANCoderConfiguration();
        config.sensorCoefficient = 2 * 3.14 / 4096.0;
        config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        canCoder.configAllSettings(config);
    }

    // Run this in periodic for all swerves. It just keeps stuff up to speed.
    public void update()  {
        // // Check for loop around
        // loopAround = (Math.abs(canCoder.getAbsolutePosition() - requestedAngle) > (3 * Math.PI / 2)); 

        // // Cancoder variable to help with loop around.
        // canCoderModified = canCoder.getAbsolutePosition();
        // canCoderModified -= offset;
        // if(canCoderModified < 0) {
        //     canCoderModified += 2 * Math.PI;
        // }
        // if(loopAround) {
        //     if(canCoderModified > Math.PI) {
        //         canCoderModified -= 2 * Math.PI;
        //     }
        //     if(requestedAngle > Math.PI) {
        //         requestedAngle -= 2 * Math.PI;
        //     }
        // }
        

        // If the wheel is facing the other direction, invert speed
        // if(Math.abs(canCoderModified - requestedAngle) > Math.PI / 2) {
        //     isInverted = -1;
        // } else {
        //     isInverted = 1;
        // }

        // Set the speed
        drive.set(maxdrive * isInverted * driveSpeed);

        // // Calculate dif
        // dif = canCoderModified - requestedAngle;

        // // Protect against 0/0
        // if(dif == 0) {
        //     dif = 0.0001;
        // }
        // turn.set(ControlMode.Position, (requestedAngle) / (Math.PI * 4) * 4096 * 150 / 7);

        turn.set(ControlMode.Position, SmartDashboard.getNumber("turn", 0));

        // Set the turn
        // turn.set(maxturn * dif / Math.abs(dif) / (1 + Math.pow(Math.E, (m * Math.abs(dif) + b))));
        System.out.println(turnID);
        System.out.println((requestedAngle) / (Math.PI * 4) * 4096 * 150 / 7);
    }

    // Takes x, y vector on desired robot direction
    public void setCartesian(double x, double y) {

        // Calculate the requested angle
        requestedAngle = Math.atan2(y, x);

        // Set the speed
        driveSpeed = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        
    }

    public void setPolar(double ang, double spe) {

        // Calculate the requested angle
        requestedAngle = ang;

        // Set the speed
        driveSpeed = spe;

    }
}
