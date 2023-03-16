// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;


import edu.wpi.first.wpilibj2.command.InstantCommand;
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

public class Balance_Auto extends SequentialCommandGroup 
{
  /** Creates a new One_Ball_Auto. */
  private final Arm m_arm;
  private final Wrist m_wrist;
  private final Intake m_intake;
  private final Extend m_extend;
  private final Swerve s_Swerve;

  public Balance_Auto(Arm arm, Intake intake, Extend extend, Wrist wrist, Swerve swerve) 
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
      new Auton_TeleopSwerve(s_Swerve, -0.3, 0, 0, 3, 0, false)
     // new Auton_Balance_TeleopSwerve(s_Swerve, -0.3, 0, 0)
       );

  }
}