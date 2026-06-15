package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface ModuleIO {
    
    public void setModuleState(SwerveModuleState targetState); 
    public Rotation2d getModuleAngle(); 
    public AngularVelocity getModuleSpeed(); 
    public void setDriveVoltage(Voltage targetVoltage); 
    public void setTurnVoltage(Voltage targetVoltage); 
    public SwerveModulePosition getModulePosition(); 
    public SwerveModuleState getModuleState(); 

}
