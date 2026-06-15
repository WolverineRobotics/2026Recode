package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private final ModuleIO frontLeftModule; 
    private final ModuleIO frontRightModule; 
    private final ModuleIO backLeftModule; 
    private final ModuleIO backRightModule; 

    private final GyroIO gyro; 

    private final SwerveDriveKinematics m_SwerveDriveKinematics; 
    private final SwerveDriveOdometry m_SwerveOdometry; 

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

        m_SwerveOdometry = new SwerveDriveOdometry(
            m_SwerveDriveKinematics, 
            getAngle(), 
            new SwerveModulePosition[] {
                frontLeftModule.getModulePosition(), 
                frontRightModule.getModulePosition(), 
                backLeftModule.getModulePosition(), 
                backRightModule.getModulePosition()
            }
        ); 

    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getRobotPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );


    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }


    }
    

    public Command driveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier rotInput) {
        // Compute module states each execution so controller inputs are polled continuously
        return run(() -> {
            ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xInput.getAsDouble(),
                yInput.getAsDouble(),
                rotInput.getAsDouble(),
                gyro.getRobotAngle()
            );

            SwerveModuleState[] targetStates = m_SwerveDriveKinematics.toSwerveModuleStates(targetSpeeds);

            frontLeftModule.setModuleState(targetStates[0]);
            frontRightModule.setModuleState(targetStates[1]);
            backLeftModule.setModuleState(targetStates[2]);
            backRightModule.setModuleState(targetStates[3]);
        });
    }

    private void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] targetStates = m_SwerveDriveKinematics.toSwerveModuleStates(speeds);

        frontLeftModule.setModuleState(targetStates[0]);
        frontRightModule.setModuleState(targetStates[1]);
        backLeftModule.setModuleState(targetStates[2]);
        backRightModule.setModuleState(targetStates[3]);

    }

    public Pose2d getRobotPose() {
        return m_SwerveOdometry.getPoseMeters(); 
    }

    public void resetPose(Pose2d targetPose) {
        m_SwerveOdometry.resetPose(targetPose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_SwerveDriveKinematics.toChassisSpeeds(
            new SwerveModuleState[] {
                frontLeftModule.getModuleState(),
                frontRightModule.getModuleState(),
                backLeftModule.getModuleState(),
                backRightModule.getModuleState()
            }
        ); 
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
