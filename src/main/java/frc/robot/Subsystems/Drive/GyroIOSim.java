package frc.robot.Subsystems.Drive;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroIOSim implements GyroIO {
    private static final GyroSimulation gyroSimulation = new GyroSimulation(3, 0.05); 

    public GyroIOSim() {
       
    }


    @Override
    public Rotation2d getRobotAngle() {
        return gyroSimulation.getGyroReading(); 
    }
    @Override
    public AngularVelocity getRobotAngularVelocity() {
        return gyroSimulation.getMeasuredAngularVelocity(); 
    }

    public static GyroSimulation getGyroSim() {
        return gyroSimulation; 
    }
    
}
