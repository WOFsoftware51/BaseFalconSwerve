// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.Auton_Approach_Bridge_Forward_Swerve;
import frc.robot.commands.Auton_Arm;
import frc.robot.commands.Auton_Climb_Bridge_Forward_Swerve;
import frc.robot.commands.Auton_TeleopSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extend;
import frc.robot.subsystems.Intake;

import frc.robot.subsystems.Wrist;

public class ScoreCone_BalancePlus3 extends SequentialCommandGroup 
{
  /** Creates a new One_Ball_Auto. */

  public ScoreCone_BalancePlus3(Arm arm, Intake intake, Extend extend, Wrist wrist, Swerve swerve) 
   {
    addRequirements(arm);
    addRequirements(wrist);
    addRequirements(extend);
    addRequirements(intake);
    addRequirements(swerve);

    
    addCommands(
      new InstantCommand(() -> swerve.zeroGyro()),
      new AutoScoreCone(arm, intake, extend, wrist),
      new ParallelRaceGroup
      (
        new Auton_TeleopSwerve(swerve, -0.35, 0, 0, 150),
        new Auton_Arm(arm, 90)
      ),
      new Auton_Approach_Bridge_Forward_Swerve(swerve, 0.3, 0, 0, 150),
      new Auton_Climb_Bridge_Forward_Swerve(swerve, 0.2, 0, 0, 100),
      new Auton_Arm(arm, 0)
       );

  }
}