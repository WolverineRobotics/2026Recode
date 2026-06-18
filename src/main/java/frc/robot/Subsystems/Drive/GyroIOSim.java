package frc.robot.Subsystems.Drive;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation; 

    public GyroIOSim(GyroSimulation gyroSimulation) {
       this.gyroSimulation = gyroSimulation;
    }


    @Override
    public Rotation2d getRobotAngle() {
        return gyroSimulation.getGyroReading(); 
    }
    @Override
    public AngularVelocity getRobotAngularVelocity() {
        return gyroSimulation.getMeasuredAngularVelocity(); 
    }

    public GyroSimulation getGyroSim() {
        return gyroSimulation; 
    }
    
}
