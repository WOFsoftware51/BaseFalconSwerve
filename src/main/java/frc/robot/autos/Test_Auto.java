// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auton_TeleopSwerve;
import frc.robot.subsystems.Swerve;

public class Test_Auto extends SequentialCommandGroup 
{
  /** Creates a new One_Ball_Auto. */
  private final Swerve s_Swerve;




  public Test_Auto(Swerve swerve) 
   {
    this.s_Swerve = swerve;
    addRequirements(swerve);

    // PathPlannerTrajectory examplePath = PathPlanner.loadPath("L_Shape", new PathConstraints(4.5, 4));


    addCommands(
      // new InstantCommand(() -> s_Swerve.zeroGyro()),
      // s_Swerve.followTrajectoryCommand(examplePath, true)
      new Auton_TeleopSwerve(s_Swerve, -0.2, 0, 0, 1.5, 0, false),
      new Auton_TeleopSwerve(s_Swerve, 0, (0.2), 0, 1.5, 0, false),
      new Auton_TeleopSwerve(s_Swerve, (0.2), 0, 0, 1.5, 0, false),
      new Auton_TeleopSwerve(s_Swerve, 0, -0.2, 0, 1.5, 0, false)

       );
  }
}