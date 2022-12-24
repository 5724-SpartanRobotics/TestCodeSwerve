package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.simulation.ADIS16448_IMUSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Constant.DebugLevel;
import frc.robot.Subsystems.Constant.DebugSetting;
import frc.robot.Subsystems.Constant.DriveConstants;

public class DriveTrainSubsystemRick extends SubsystemBase implements DriveTrainInterface {
        // Swerve modules
        private SwerveModule LF;
        private SwerveModule RF;
        private SwerveModule LB;
        private SwerveModule RB;
        private Rotation2d lastUpdatedGyroHeading;
    
        // Gyro for now
        private ADIS16448_IMU gyro;
        private ADIS16448_IMUSim gyroSim;

        private SwerveDriveKinematics swerveDriveKinematics;
        private SwerveDriveOdometry swerveDriveOdometry;
        private Pose2d robotPose;
    
        public DriveTrainSubsystemRick() {
            gyro = new ADIS16448_IMU();
            UpdateGyro();
            LF = new SwerveModule(DriveConstants.LFTurnMotor, DriveConstants.LFDriveMotor, DriveConstants.LFCanID, DriveConstants.LFOff, "LF", this);
            RF = new SwerveModule(DriveConstants.RFTurnMotor, DriveConstants.RFDriveMotor, DriveConstants.RFCanID, DriveConstants.RFOff, "RF", this);
            LB = new SwerveModule(DriveConstants.LBTurnMotor, DriveConstants.LBDriveMotor, DriveConstants.LBCanID, DriveConstants.LBOff, "LB", this);
            RB = new SwerveModule(DriveConstants.RBTurnMotor, DriveConstants.RBDriveMotor, DriveConstants.RBCanID, DriveConstants.RBOff, "RB", this);
            swerveDriveKinematics = new SwerveDriveKinematics(DriveConstants.LFLocation, DriveConstants.RFLocation, DriveConstants.LBLocation, DriveConstants.RBLocation);
            robotPose = new Pose2d(new Translation2d(4.0, 5.0), new Rotation2d());//starting pose doesn't really matter. We will call reset based on robot initial field position.
            swerveDriveOdometry = new SwerveDriveOdometry(swerveDriveKinematics, getGyroHeading(), robotPose);
        }

        public Rotation2d getGyroHeading(){
            return new Rotation2d(); //lastUpdatedGyroHeading;
        }

        @Override
        public void periodic(){
            //the gyro getGyroAngleY returns positive values as the robot turns clockwise. We want negative for clockwise
            LF.periodic();
            LB.periodic();
            RF.periodic();
            RB.periodic();
            if (DebugSetting.TraceLevel == DebugLevel.Verbose){
                SmartDashboard.putNumber("Gyro Heading Deg", getGyroHeading().getDegrees());
            }
            UpdateGyro();
            robotPose = swerveDriveOdometry.update(getGyroHeading(), LF.getState(), RF.getState(), LB.getState(), RB.getState());
        }

        private void UpdateGyro() {
            lastUpdatedGyroHeading = Rotation2d.fromDegrees(-gyro.getGyroAngleY());
        }

        public void drive(Translation2d translation, double rotation){
            SwerveModuleState[] swerveModStates = swerveDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getGyroHeading()));
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModStates, DriveConstants.maxRobotSpeedmps);
            LF.setDesiredState(swerveModStates[0]);
            RF.setDesiredState(swerveModStates[1]);
            LB.setDesiredState(swerveModStates[2]);
            RB.setDesiredState(swerveModStates[3]);
        }

        public void simulationInit()
        {
            gyroSim = new ADIS16448_IMUSim(gyro);
            LF.simulateInit();
            RF.simulateInit();
            LB.simulateInit();
            RB.simulateInit();
            gyroSim.setGyroAngleY(0);
        }
        @Override
        public void simulationPeriodic(){
            if (gyroSim == null)
                simulationInit();
            gyroSim.setGyroAngleY(Units.radiansToDegrees(0.0));
            LF.simulatePeriodic();
            RF.simulatePeriodic();
            LB.simulatePeriodic();
            RB.simulatePeriodic();
            }
}
