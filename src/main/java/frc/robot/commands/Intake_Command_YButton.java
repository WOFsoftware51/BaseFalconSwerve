// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Global_Variables;
//import frc.robot.subsystems.CANdle;
import frc.robot.subsystems.Intake;


public class Intake_Command_YButton extends CommandBase 
{

  private final Intake m_intake;
  private int count = 0;
  private boolean left_bumper = false;
  private boolean right_bumper = false;

  /** Creates a new Intake. */
  // private final CANdle m_candle;
  

  /** Creates a new Intake. */
  public Intake_Command_YButton(Intake intake) 
  {
    this.m_intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
   // m_candle.CANdle_Intake(lednum);
   m_intake.intake_init();
   Global_Variables.have_game_piece = false;
   count = 0;
   left_bumper = Global_Variables.left_bumper;
   right_bumper = Global_Variables.right_bumper; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    SmartDashboard.putNumber("Intake Speed", m_intake.Intake_Speed());
    SmartDashboard.putNumber("Intake Current", m_intake.Intake_Current());

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

    if(right_bumper==true)
    {
    if(Global_Variables.have_game_piece)
    {
      m_intake.Intake_Slow();
    }
    else
    {
      m_intake.Intake_On();
    }
  }
  else
  {
    m_intake.Intake_Off();
  }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
   // m_intake.Intake_Slow();
   m_intake.Intake_Slow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
