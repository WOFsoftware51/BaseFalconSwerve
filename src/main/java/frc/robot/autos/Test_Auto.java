// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;


import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.Auton_Arm_Extend;
import frc.robot.commands.Auton_Intake;
import frc.robot.commands.Auton_TeleopSwerve;
import frc.robot.commands.Auton_Wait;
import frc.robot.commands.ScoreMiddle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extend;
import frc.robot.subsystems.Intake;

import frc.robot.subsystems.Wrist;

public class Test_Auto extends SequentialCommandGroup 
{
  /** Creates a new One_Ball_Auto. */
  private final Arm m_arm;
  private final Wrist m_wrist;
  private final Intake m_intake;
  private final Extend m_extend;
  private final Swerve s_Swerve;




  public Test_Auto(Arm arm, Intake intake, Extend extend, Wrist wrist, Swerve m_Swerve) 
   {
    this.m_arm = arm;
    addRequirements(arm);
    this.m_wrist = wrist;
    addRequirements(wrist);
    this.m_extend = extend;
    addRequirements(extend);
    this.m_intake = intake;
    addRequirements(intake);
    this.s_Swerve = m_Swerve;
    addRequirements(m_Swerve);


    addCommands(
      new InstantCommand(() -> s_Swerve.zeroGyro()),
    //  s_Swerve.followTrajectoryCommand(s_Swerve.examplePath, true),
      //s_Swerve.followTrajectoryCommand(s_Swerve.examplePath2, false),
      new Auton_TeleopSwerve(s_Swerve, 0.1, 0, 0, 0.3, 0, false),
      new ParallelRaceGroup(new Auton_Arm_Extend(m_extend, Constants.ARM_HUMAN_PLAYER_LOAD), new ScoreMiddle(m_arm, Constants.WRIST_HUMAN_PLAYER_LOAD, m_wrist, Constants.WRIST_SCORE), new Auton_Wait(100)),
      new Auton_Intake(m_intake, 20, false),
      new ParallelRaceGroup(new Auton_Arm_Extend(m_extend, 0), new ScoreMiddle(m_arm, 0, m_wrist, 0), new Auton_Wait(100)),
      new Auton_TeleopSwerve(s_Swerve, -0.1, 0, 0, 0.3, 0, false)
      );
  }
}