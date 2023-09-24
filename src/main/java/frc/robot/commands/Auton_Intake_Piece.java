// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Global_Variables;
import frc.robot.subsystems.Intake;

public class Auton_Intake_Piece extends CommandBase 
{
  /** Creates a new Auton_Wait. */

  private double time = 0.0;
  private double wait = 0.0;
  private double counter = 0.0;
  private double count = 0.0;
  private boolean end = false;
  private Intake m_intake;
  private boolean toggle;

  public Auton_Intake_Piece(Intake intake, double wait, boolean toogle) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.m_intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    end = false;
    counter = 0.0;
    Global_Variables.have_game_piece = false;
    m_intake.intake_init();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    SmartDashboard.putNumber("Auton Counter", counter);
    //SmartDashboard.putNumber("Auton Time", time);

    if(toggle == true)
    {
      if(m_intake.Intake_Current() > 12 && m_intake.Intake_Speed() < 3)
      {
        count++;
        if(count > 10)
        {
          Global_Variables.have_game_piece = true;
        }
      }
      else
      {
        count = 0;
      }
  
      if( Global_Variables.have_game_piece)
      {
        m_intake.Intake_Slow();
      }
      else
      {
        m_intake.Intake_On();
      }

      if(Global_Variables.have_game_piece == true)
      {
        counter++;
      }
  
      if(counter>10)
      {
        end = true;
      }
    } 
    else if(toggle == false)
    {
      m_intake.Intake_Reverse();
    }

    if(time>wait)
    {
      end = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
