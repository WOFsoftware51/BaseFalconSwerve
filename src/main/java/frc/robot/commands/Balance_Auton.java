// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Auton_Subsystem;
import frc.robot.subsystems.Swerve;

public class Balance_Auton extends CommandBase 
{
  Auton_Subsystem m_auto;
  Swerve s_Swerve;
  private int wait_length = 0;
  
  TrajectoryConfig config =
  new TrajectoryConfig(
          Constants.AutoConstants.kBalanceSpeedMetersPerSecond,
          Constants.AutoConstants.kBalanceAccelerationMetersPerSecondSquared)
      .setKinematics(Constants.Swerve.swerveKinematics);
  
  //goes foward a little
    Trajectory balanceFixForward =
  TrajectoryGenerator.generateTrajectory(
   // Start at the origin facing the +X direction
   new Pose2d(1.6764, 0, new Rotation2d(0)),
   List.of(),
   new Pose2d(1.6764, -0.1, new Rotation2d(0)),
   config);

   //goes back a little
   Trajectory balanceFixBackwards =
   TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(1.6764, 0, new Rotation2d(0)),
    List.of(),
    new Pose2d(1.6764, + 0.1, new Rotation2d(0)),
    config);
    
    Trajectory doesNothing =
    TrajectoryGenerator.generateTrajectory(
     // Start at the origin facing the +X direction
     new Pose2d(1.6764, 0, new Rotation2d(0)),
     List.of(),
     new Pose2d(1.6764, 0, new Rotation2d(0)),
     config);

    Trajectory afd = TrajectoryGenerator.generateTrajectory(List.of(), config);
   
  /** Creates a new Balance_Auton. */
  public Balance_Auton(Swerve swerve, Auton_Subsystem auto) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(auto);
    this.m_auto = auto;
    addRequirements(swerve);
    this.s_Swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    /* 
    if(s_Swerve.getPitch() < 0)
    {
      m_auto.trajectory = balanceFixForward;
    }
    else if(s_Swerve.getPitch() > 0)
    {
      m_auto.trajectory = balanceFixBackwards;
    }
    else if(s_Swerve.getPitch() == 0)
    {
     m_auto.trajectory = doesNothing;
    }
    */
    wait_length++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if(wait_length == 1)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}
