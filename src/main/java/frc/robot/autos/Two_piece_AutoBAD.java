// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.AutonSwerve;
import frc.robot.commands.Auton_Approach_Cube_Swerve;
import frc.robot.commands.Auton_Arm_Extend;
import frc.robot.commands.Auton_Intake;
import frc.robot.commands.Auton_TeleopSwerve;
import frc.robot.commands.Auton_Wait;
import frc.robot.commands.ScoreMiddle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extend;
import frc.robot.subsystems.Intake;

import frc.robot.subsystems.Wrist;

public class Two_piece_AutoBAD extends SequentialCommandGroup 
{
  /** Creates a new One_Ball_Auto. */
  private final Arm m_arm;
  private final Wrist m_wrist;
  private final Intake m_intake;
  private final Extend m_extend;
  private final Swerve s_Swerve;

  public Two_piece_AutoBAD(Arm arm, Intake intake, Extend extend, Wrist wrist, Swerve swerve) 
   {
    this.m_arm = arm;
    addRequirements(arm);
    this.m_wrist = wrist;
    addRequirements(wrist);
    this.m_extend = extend;
    addRequirements(extend);
    this.m_intake = intake;
    addRequirements(intake);
    this.s_Swerve = swerve;
    addRequirements(swerve);

    
    addCommands(
      new InstantCommand(() -> s_Swerve.zeroGyro()),
      new ParallelRaceGroup
      (
        new Auton_Arm_Extend(m_extend, Constants.EXTEND_SCORE_HIGH), 
        new ScoreMiddle(m_arm, Constants.ARM_SCORE_HIGH, m_wrist, Constants.WRIST_SCORE), 
        new Auton_Wait(100)
      ),
      new Auton_Intake(m_intake, 20, false),
      new ParallelRaceGroup
      (
        new Auton_Arm_Extend(m_extend, 0), 
        new ScoreMiddle(m_arm, Constants.ARM_PICKUP_CUBE + 10, m_wrist, Constants.WRIST_PICKUP_CUBE), 
        new AutonSwerve(s_Swerve, -0.2, -0.2*m_arm.getAllianceColor(), 0, 35)
      ),
      new ParallelRaceGroup
      (
        new Auton_Arm_Extend(m_extend, 0), 
        new Auton_Approach_Cube_Swerve(s_Swerve, -0.45, 0, 0, 150),
        new ScoreMiddle(m_arm, Constants.ARM_PICKUP_CUBE + 5, m_wrist, Constants.WRIST_PICKUP_CUBE), 
        new Auton_Intake(m_intake, 150, true)
      ),
      new ParallelRaceGroup
      (
        new Auton_Arm_Extend(m_extend, 0), 
        new Auton_Approach_Cube_Swerve(s_Swerve, -0.3, 0, 0, 100),
        new ScoreMiddle(m_arm, Constants.ARM_PICKUP_CUBE, m_wrist, Constants.WRIST_PICKUP_CUBE), 
        new Auton_Intake(m_intake, 100, true)
      ),
      new ParallelRaceGroup
      (
        new Auton_Arm_Extend(m_extend, 0), 
        new ScoreMiddle(m_arm, 0, m_wrist, 0), 
        new AutonSwerve(s_Swerve, 0.45, 0, 0, 150)
      ),
      new AutonSwerve(s_Swerve, 0.25, 0, 0, 100),

      new ParallelRaceGroup
      (
        new Auton_Arm_Extend(m_extend, Constants.EXTEND_SCORE_HIGH), 
        new ScoreMiddle(m_arm, Constants.ARM_SCORE_HIGH, m_wrist, Constants.WRIST_SCORE), 
        new Auton_Wait(120),
        new AutonSwerve(s_Swerve, 0, 0, 0, 120)
      ),
      new Auton_Intake(m_intake, 20, false),
      new ParallelRaceGroup
      (
        new Auton_Arm_Extend(m_extend, 0), 
        new ScoreMiddle(m_arm, 0, m_wrist, 0), 
        new Auton_Wait(100)
      )

      );
  }
}