// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Drive.GyroIOSim;
import frc.robot.Subsystems.Drive.ModuleIO;
import frc.robot.Subsystems.Drive.ModuleIOSim;

public class RobotContainer {

  private final DriveSubsystem m_DriveSubsystem; 
  private final CommandXboxController driveController; 
  private final DriveTrainSimulationConfig simulationConfig; 
  private final SwerveDriveSimulation swerveSimulation; 
  private final StructPublisher<Pose2d> swervePosePublisher; 

  public RobotContainer() {
    m_DriveSubsystem = new DriveSubsystem(
      new ModuleIO[] {
        new ModuleIOSim(), 
        new ModuleIOSim(), 
        new ModuleIOSim(), 
        new ModuleIOSim()
      }, 
      new GyroIOSim( )
    ); 

    driveController = new CommandXboxController(0); 
    simulationConfig = new DriveTrainSimulationConfig(
      Pounds.of(81), 
      Inches.of(15),
      Inches.of(16), 
      Inches.of(14),
      Inches.of(14), 
      () -> GyroIOSim.getGyroSim(),
      () -> ModuleIOSim.getModuleSim()
      ); 

      swerveSimulation = new SwerveDriveSimulation(simulationConfig, new Pose2d(0.1, 0.1, new Rotation2d())); 
      SimulatedArena.getInstance().addDriveTrainSimulation(swerveSimulation);

      swervePosePublisher = NetworkTableInstance.getDefault().getStructTopic("Bot Pose", Pose2d.struct).publish(); 

    configureBindings();


  }

  private void configureBindings() {
    m_DriveSubsystem.setDefaultCommand(
      m_DriveSubsystem.driveCommand(
        () -> driveController.getLeftX(),
        () -> driveController.getLeftY(),
        () -> driveController.getRightX()
      )
    );
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Example Auto"); 
  }

  public void updateSimulation() {
    SimulatedArena.getInstance().simulationPeriodic();
    swerveSimulation.setRobotSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(driveController.getLeftX(), driveController.getLeftY(), driveController.getRightX(), m_DriveSubsystem.getAngle()));
    swervePosePublisher.set(swerveSimulation.getSimulatedDriveTrainPose());  
  }
}
