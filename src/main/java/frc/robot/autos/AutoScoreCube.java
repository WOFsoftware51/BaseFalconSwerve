// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;


import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.Auton_Arm_Extend;
import frc.robot.commands.Auton_Intake;
import frc.robot.commands.Auton_Wait;
import frc.robot.commands.ScoreMiddle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extend;
import frc.robot.subsystems.Intake;

import frc.robot.subsystems.Wrist;

public class AutoScoreCube extends SequentialCommandGroup 
{
  /** Creates a new One_Ball_Auto. */
  private final Arm m_arm;
  private final Wrist m_wrist;
  private final Intake m_intake;
  private final Extend m_extend;
  private final Swerve s_Swerve;

  public AutoScoreCube(Arm arm, Intake intake, Extend extend, Wrist wrist, Swerve swerve) 
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
      new ParallelRaceGroup(
        new Auton_Arm_Extend(m_extend,  0), 
        new ScoreMiddle(m_arm, 0, m_wrist, Constants.WRIST_SCORE_HIGH_AUTON),
        new Auton_Wait(100)
      ),
      new Auton_Intake(intake, 20, false),
      new ParallelRaceGroup(
        new RunCommand(() -> m_intake.Intake_Reverse_Fast()),
        new WaitCommand(20)
      )
      );
  }
}