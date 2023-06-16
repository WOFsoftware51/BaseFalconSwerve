// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class Test_Auto extends SequentialCommandGroup 
{
  /** Creates a new One_Ball_Auto. */
  private final Swerve s_Swerve;




  public Test_Auto(Swerve m_Swerve) 
   {
    this.s_Swerve = m_Swerve;
    addRequirements(m_Swerve);

    PathPlannerTrajectory examplePath = PathPlanner.loadPath("L_Shape", new PathConstraints(4.5, 4));


    addCommands(
      new InstantCommand(() -> s_Swerve.zeroGyro()),
      s_Swerve.followTrajectoryCommand(examplePath, true)
      );
  }
}