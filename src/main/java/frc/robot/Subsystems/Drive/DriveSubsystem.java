package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private final ModuleIO frontLeftModule; 
    private final ModuleIO frontRightModule; 
    private final ModuleIO backLeftModule; 
    private final ModuleIO backRightModule; 

    private final GyroIO gyro; 

    private final SwerveDriveKinematics m_SwerveDriveKinematics; 

    public DriveSubsystem(ModuleIO[] modules, GyroIO gyro) {
        
        this.frontLeftModule = modules[0]; 
        this.frontRightModule = modules[1]; 
        this.backLeftModule = modules[2]; 
        this.backRightModule = modules[3]; 

        this.gyro = gyro; 

        this.m_SwerveDriveKinematics = new SwerveDriveKinematics(
            DriveConstants.frontLeftLocation,
            DriveConstants.frontRightLocation, 
            DriveConstants.backLeftLocation, 
            DriveConstants.backRightLocation
        ); 

    }
    

    public Command driveCommand(double xInput, double yInput, double rotInput) {

        ChassisSpeeds targetSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(xInput, yInput, rotInput, gyro.getRobotAngle()); 
        SwerveModuleState[] targetStates = m_SwerveDriveKinematics.toSwerveModuleStates(targetSpeeds); 

        return run(() -> {
            frontLeftModule.setModuleState(targetStates[0]);
            frontRightModule.setModuleState(targetStates[1]);
            backLeftModule.setModuleState(targetStates[2]);
            backRightModule.setModuleState(targetStates[3]);
        });
        
        
    }

    public GyroIO getGyroIO() {
        return gyro; 
    }

    public ModuleIO[] getModules() {
        return new ModuleIO[] {
            frontLeftModule, 
            frontRightModule, 
            backLeftModule, 
            backRightModule
        }; 
    }

    public Rotation2d getAngle() {
        return gyro.getRobotAngle(); 
    }


}
