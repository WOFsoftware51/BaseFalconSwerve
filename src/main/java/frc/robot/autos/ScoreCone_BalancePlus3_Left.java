// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.commands.AutonSwerve;
import frc.robot.commands.Auton_Approach_Bridge_Reverse_Swerve;
import frc.robot.commands.Auton_Approach_Bridge_Reverse_Swerve_Right;
import frc.robot.commands.Auton_Approach_Flat_Swerve;
import frc.robot.commands.Auton_Arm_Extend;
import frc.robot.commands.Auton_Climb_Bridge_Backward_Swerve;
import frc.robot.commands.Auton_Intake;
import frc.robot.commands.Auton_TeleopSwerve;
import frc.robot.commands.Auton_Wait;
import frc.robot.commands.ScoreMiddle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extend;
import frc.robot.subsystems.Intake;

import frc.robot.subsystems.Wrist;

public class ScoreCone_BalancePlus3_Left extends SequentialCommandGroup 
{

  private final Arm m_arm;
  private final Wrist m_wrist;
  private final Extend m_extend;
  private final Swerve s_Swerve;
  private final Intake m_intake;
  /** Creates a new One_Ball_Auto. */

  public ScoreCone_BalancePlus3_Left(Arm arm, Intake intake, Extend extend, Wrist wrist, Swerve swerve) 
   {
    this.s_Swerve = swerve;
    addRequirements(s_Swerve);
    this.m_arm = arm;
    addRequirements(m_arm);
    this.m_wrist = wrist;
    addRequirements(m_wrist);
    this.m_extend = extend;
    addRequirements(m_extend);
    this.m_intake = intake;
    addRequirements(m_intake);


    
    addCommands
    (
      new InstantCommand(() -> s_Swerve.zeroGyro()),
      new ParallelRaceGroup
      (
        new Auton_Arm_Extend(m_extend, Constants.EXTEND_SCORE_HIGH), 
        new ScoreMiddle(m_arm, Constants.ARM_SCORE_HIGH-5, m_wrist, Constants.WRIST_SCORE), 
        new Auton_Wait(110)),
      new Auton_Intake(m_intake, 20, false),
      new ParallelRaceGroup
      (
        new Auton_Arm_Extend(m_extend, 0), 
        new ScoreMiddle(m_arm, -90, m_wrist, 80), 
        new Auton_Wait(50)),
      new ParallelRaceGroup
        (
          new Auton_Arm_Extend(m_extend, 0), 
          new ScoreMiddle(m_arm, -90, m_wrist, 45), 
          new Auton_TeleopSwerve(s_Swerve, -0.6, 0, 0, 2.3, 0, false)
        ),
        
      new Auton_Approach_Bridge_Reverse_Swerve(s_Swerve, -0.4, 100),     //TODO LOWER 250 TIMER FOR ALL INSTANCES OF 250
      new Auton_Approach_Flat_Swerve(s_Swerve, -0.3, 100), 
      new ParallelRaceGroup
      (
        new Auton_Arm_Extend(m_extend, 0), 
        new ScoreMiddle(m_arm, Constants.ARM_PICKUP_CUBE, m_wrist, Constants.WRIST_PICKUP_CUBE), 
        new AutonSwerve(s_Swerve, -0.0, 0, 0, 100),
        new Auton_Intake(intake, 40, true)
      ),
      new ParallelRaceGroup
      (
        new Auton_Arm_Extend(m_extend, 0), 
        new ScoreMiddle(m_arm, Constants.ARM_PICKUP_CUBE, m_wrist, Constants.WRIST_PICKUP_CUBE), 
        new AutonSwerve(s_Swerve, -0.4, 0, 0, 100),
        new Auton_Intake(intake, 50, true)
      ),
      new ParallelRaceGroup
      (
        new Auton_Arm_Extend(m_extend, 0), 
        new ScoreMiddle(m_arm, 0, m_wrist, 90),
        new Auton_Approach_Bridge_Reverse_Swerve_Right(s_Swerve, 0.4, 250)      
      ),
      new ParallelRaceGroup
      (
        new Auton_Arm_Extend(m_extend, 0), 
        new ScoreMiddle(m_arm, 0, m_wrist, 90),
        new Auton_Climb_Bridge_Backward_Swerve(s_Swerve, 0.4, 250)      
      ), 

      new ParallelRaceGroup
      (
        new Auton_Arm_Extend(m_extend, 0), 
        new ScoreMiddle(m_arm, -5, m_wrist, 90), 
        new AutonSwerve(s_Swerve, -0.2, 0, 0, 100),
        new RunCommand(() -> m_intake.Intake_Reverse_Fast())
    ),
    new ParallelRaceGroup(
      new Auton_Intake(m_intake, 100, false),
      new AutonSwerve(s_Swerve, 0, 0, 0, 100)
    )
    );

  }
}
