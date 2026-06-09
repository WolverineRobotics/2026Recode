package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.DriveConstants;

public class ModuleIOSim implements ModuleIO {
    

    private final SimulatedMotorController.GenericMotorController driveMotor; 
    private final SimulatedMotorController.GenericMotorController turnMotor; 

    private static final SwerveModuleSimulationConfig moduleSimConfig = new SwerveModuleSimulationConfig(
                DCMotor.getNEO(1), 
                DCMotor.getNEO(1),
                DriveConstants.driveGearRatio,
                DriveConstants.turnGearRatio, 
                Volts.of(0.1),
                Volts.of(0.1),
                DriveConstants.wheelRadius, 
                DriveConstants.steerMOI,
                 1.2
            ); 

    private static final SwerveModuleSimulation moduleSimulation = new SwerveModuleSimulation(moduleSimConfig);

    private final SimpleMotorFeedforward driveFeedForwardController; 
    private final PIDController driveFeedBackController; 
    private final PIDController turnController; 


    public ModuleIOSim() {   

        this.driveMotor = moduleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(60)); 
        this.turnMotor = moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(20)); 


        driveFeedForwardController = new SimpleMotorFeedforward(
            DriveConstants.driveKsSim, DriveConstants.driveKvSim, DriveConstants.driveKaSim 
        ); 

        driveFeedBackController = new PIDController(
            DriveConstants.driveKpSim, DriveConstants.driveKiSim, DriveConstants.driveKdSim
        ); 

        turnController = new PIDController(
            DriveConstants.turnKpSim, DriveConstants.turnKiSim, DriveConstants.turnKdSim
        ); 
    }

    @Override
    public void setModuleState(SwerveModuleState targetState) {
        turnMotor.requestVoltage(Volts.of(
            turnController.calculate(getModuleAngle().getDegrees(), targetState.angle.getDegrees())
        ));

        driveMotor.requestVoltage(Volts.of(
            driveFeedForwardController.calculate(targetState.speedMetersPerSecond / DriveConstants.wheelRadius.in(Meters))
            + driveFeedBackController.calculate(
                getModuleSpeed().in(RadiansPerSecond),
                targetState.speedMetersPerSecond / DriveConstants.wheelRadius.in(Meters)
                )
        ));
    }

    public  SwerveModuleSimulation getModuleSimConfig() {
        return moduleSimulation; 
    }

    @Override
    public Rotation2d getModuleAngle() {
        return moduleSimulation.getSteerAbsoluteFacing(); 
    }

    @Override
    public AngularVelocity getModuleSpeed() {
        return moduleSimulation.getDriveWheelFinalSpeed(); 
    }

    @Override
    public void setDriveVoltage(Voltage targetVoltage) {
        this.driveMotor.requestVoltage(targetVoltage);
    }

    @Override
    public void setTurnVoltage(Voltage targetVoltage) {
        this.turnMotor.requestVoltage(targetVoltage);
    }

    public static SwerveModuleSimulation getModuleSim() {
        return moduleSimulation; 
    }
    
}
