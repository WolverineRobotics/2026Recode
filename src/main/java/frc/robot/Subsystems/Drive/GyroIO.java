package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public interface GyroIO {

    public Rotation2d getRobotAngle(); 
    public AngularVelocity getRobotAngularVelocity(); 
    
}
