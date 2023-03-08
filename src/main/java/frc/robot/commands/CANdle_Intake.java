// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Global_Variables;
import frc.robot.subsystems.CANdle_Subsystem;
import frc.robot.Constants;

public class CANdle_Intake extends CommandBase 
{
  /** Creates a new CANdle_Intake. */
  CANdle_Subsystem m_candle;
  private boolean end = false;
  private int count = 0;
  private int button = 0;
  private boolean left_bumper = false;
  private boolean right_bumper = false;

  public CANdle_Intake(CANdle_Subsystem candle, int Button) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_candle = candle;
    this.button = Button;
    addRequirements(candle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    left_bumper = Global_Variables.left_bumper;
    right_bumper = Global_Variables.right_bumper;
    
    m_candle.CANdle_init();
    end = false;
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
     switch(button) 
      {
      case Constants.Y_Button:   ////////////////////////////////////////////////////////////
        if(right_bumper || left_bumper)    
        {
          if(Global_Variables.have_game_piece)
          {
            m_candle.CANdle_Solid_Green();
            
            count = 1;
          }
          else if(count == 1)
          {
            count = 0;
            end = true;
          }
          else
          {
            m_candle.CANdle_Orange();
          }
        }
        break;

        case Constants.B_Button:   ////////////////////////////////////////////////////////////
        if(right_bumper || left_bumper)    
        {
          if(Global_Variables.have_game_piece)
          {
            m_candle.CANdle_Solid_Green();
            
            count = 1;
          }
          else if(count == 1)
          {
            count = 0;
            end = true;
          }
          else
          {
            m_candle.CANdle_Orange();
          }
        }
        break;

        case Constants.X_Button:   ////////////////////////////////////////////////////////////
        if(right_bumper || left_bumper)    
        {
          if(Global_Variables.have_game_piece)
          {
            m_candle.CANdle_Solid_Green();
            
            count = 1;
          }
          else if(count == 1)
          {
            count = 0;
            end = true;
          }
          else
          {
            m_candle.CANdle_Purple();
          }
        }
        break;

        case Constants.A_Button:   ////////////////////////////////////////////////////////////
        if(right_bumper || left_bumper)    
        {
          if(Global_Variables.have_game_piece)
          {
            m_candle.CANdle_Solid_Green();
            
            count = 1;
          }
          else if(count == 1)
          {
            count = 0;
            end = true;
          }
          else
          {
            m_candle.CANdle_Orange();
          }
        }
        break;

      default:   ////////////////////////////////////////////////////////////
        m_candle.CANdle_Default();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_candle.CANdle_Default();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return end;
  }
}
