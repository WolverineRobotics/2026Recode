package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;

import frc.robot.Constants.DriveConstants;

public class ModuleIOSpark implements ModuleIO {

    private final SparkMax steerMotor; 
    private final SparkMax driveMotor; 

    private final SparkAnalogSensor absoluteEncoder;     

    private final SparkMaxConfig steerConfig; 
    private final SparkMaxConfig driveConfig; 
    
    private final SimpleMotorFeedforward driveFeedForward; 

    public ModuleIOSpark(
        int steerCANID, 
        int driveCANID, 
        int encoderID, 
        double encoderOffset, 
        boolean steerInverted, 
        boolean driveInverted
    ) {
        steerMotor = new SparkMax(steerCANID, MotorType.kBrushless); 
        driveMotor = new SparkMax(driveCANID, MotorType.kBrushless); 

        absoluteEncoder = steerMotor.getAnalog(); 


        steerConfig = new SparkMaxConfig(); 
    
        steerConfig.apply(
            new ClosedLoopConfig()
            .pid(DriveConstants.turnKp, DriveConstants.turnKi, DriveConstants.turnKd)
            .feedbackSensor(FeedbackSensor.kAnalogSensor)
        ); 
        steerConfig.inverted(steerInverted); 
        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
        

        driveConfig = new SparkMaxConfig(); 

        driveConfig.apply(
            new ClosedLoopConfig()
            .pid(DriveConstants.driveKp, DriveConstants.driveKi, DriveConstants.driveKd)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)

        );
        driveConfig.inverted(driveInverted); 
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

        driveFeedForward = new SimpleMotorFeedforward(DriveConstants.driveKs, DriveConstants.driveKv, DriveConstants.driveKa); 

        
    }

    @Override
    public void setModuleState(SwerveModuleState targetState) {
        steerMotor.getClosedLoopController().setSetpoint(targetState.angle.getDegrees(), ControlType.kPosition); 
        
        driveMotor.getClosedLoopController().setSetpoint(
            targetState.speedMetersPerSecond,
            ControlType.kVelocity, 
            ClosedLoopSlot.kSlot0,
            driveFeedForward.calculate(targetState.speedMetersPerSecond)
        ); 
    }

    @Override
    public Rotation2d getModuleAngle() {
        return new Rotation2d(Degrees.of(absoluteEncoder.getPosition())); 
    }

    @Override
    public AngularVelocity getModuleSpeed() {
        return DegreesPerSecond.of(absoluteEncoder.getVelocity()); 
    }

    public LinearVelocity getModuleSpeedLinear() {
        return MetersPerSecond.of(getModuleSpeed().in(RadiansPerSecond) * DriveConstants.wheelRadius.in(Meters)); 
    }

    @Override
    public void setDriveVoltage(Voltage targetVoltage) {
        driveMotor.setVoltage(targetVoltage);
    }

    @Override
    public void setTurnVoltage(Voltage targetVoltage) {
        steerMotor.setVoltage(targetVoltage);
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            Meters.of(
                Rotations.of(driveMotor.getEncoder().getPosition()).in(Radians) * DriveConstants.wheelRadius.in(Meters)
            ), 
            getModuleAngle() 
        ); 
    }

    @Override
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getModuleSpeedLinear(), getModuleAngle()); 
    }
    
}
