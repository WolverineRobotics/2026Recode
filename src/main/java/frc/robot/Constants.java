package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

public class Constants {
    
    public static class DriveConstants {

        public static final Translation2d frontLeftLocation = new Translation2d(Inches.of(14), Inches.of(14)); 
        public static final Translation2d frontRightLocation = new Translation2d(Inches.of(14), Inches.of(-14));
        public static final Translation2d backLeftLocation = new Translation2d(Inches.of(-14), Inches.of(14));  
        public static final Translation2d backRightLocation = new Translation2d(Inches.of(14), Inches.of(14)); 

        public static final double driveKpSim = 0.000295; 
        public static final double driveKiSim = 0; 
        public static final double driveKdSim = 0.0000025; 

        public static final double driveKsSim = 0; 
        public static final double driveKvSim = 0; 
        public static final double driveKaSim = 0; 

        public static final double turnKpSim = 0.01; 
        public static final double turnKiSim = 0; 
        public static final double turnKdSim = 0.0001; 

        public static final double driveKp = 0; 
        public static final double driveKi = 0; 
        public static final double driveKd = 0; 

        public static final double driveKs = 0; 
        public static final double driveKv = 0; 
        public static final double driveKa = 0; 

        public static final double turnKp = 0; 
        public static final double turnKi = 0; 
        public static final double turnKd = 0; 

        public static final Distance wheelRadius = Inches.of(2); 
        public static final MomentOfInertia steerMOI = KilogramSquareMeters.of(0.03); 

        public static final double driveGearRatio = 6.75; 
        public static final double turnGearRatio = 21.428; 
    }

}
