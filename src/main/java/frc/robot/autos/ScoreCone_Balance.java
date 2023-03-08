// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.AutonSwerve;
import frc.robot.commands.Auton_Approach_Bridge_Forward_Swerve;
import frc.robot.commands.Auton_Climb_Bridge_Forward_Swerve;
import frc.robot.commands.Auton_Reset;
import frc.robot.commands.Auton_TeleopSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extend;
import frc.robot.subsystems.Intake;

import frc.robot.subsystems.Wrist;

public class ScoreCone_Balance extends SequentialCommandGroup 
{

  private final Arm m_arm;
  private final Wrist m_wrist;
  private final Extend m_extend;
  private final Swerve s_Swerve;
  /** Creates a new One_Ball_Auto. */

  public ScoreCone_Balance(Arm arm, Intake intake, Extend extend, Wrist wrist, Swerve s_Swerve) 
   {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    this.m_arm = arm;
    addRequirements(m_arm);
    this.m_wrist = wrist;
    addRequirements(m_wrist);
    this.m_extend = extend;
    addRequirements(m_extend);


    
    addCommands(
    //  new InstantCommand(() -> s_Swerve.zeroGyro()),
      new Auton_Reset(arm, wrist, extend, s_Swerve),
      new AutoScoreCone(arm, intake, extend, wrist),
      new Auton_Approach_Bridge_Forward_Swerve(s_Swerve, -0.35, 0, 0, 250),
      new Auton_Climb_Bridge_Forward_Swerve(s_Swerve, -0.25, 0, 0, 250),
      new Auton_TeleopSwerve(s_Swerve, 0.2, 0, 0, 50),
      new Auton_TeleopSwerve(s_Swerve, 0, 0, 0, 100)

       );

  }
}