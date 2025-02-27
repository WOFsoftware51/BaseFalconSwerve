// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class Drive_Boost extends CommandBase 
{
  /** Creates a new Boost_On. */
  Swerve s_swerve;
  private int init_counter = 0; 
  
  public Drive_Boost(Swerve swerve) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    s_swerve.Boost_On();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
  //end(isFinished());
    init_counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
      s_swerve.Boost_On();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
  if(init_counter>=1) 
    { 
      return true;
    } 
      else 
    {
     return false;
    }
  }
}
