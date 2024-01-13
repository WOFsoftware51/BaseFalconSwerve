// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.subsystems.Extend;

public class Extend_Setposition extends CommandBase 
{

  private final Extend m_extend;

  
  private double extendEncoder = 0.0;
  private int button = 0;
  private boolean left_bumper = false;
  private boolean right_bumper = false;
 
  private double extendTarget;

  /** Creates a new Arm. */
  public Extend_Setposition(Extend extend, int Button) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_extend = extend;
    addRequirements(extend);
    this.button = Button;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_extend.extend_init();
    left_bumper = Global_Variables.left_bumper;
    right_bumper = Global_Variables.right_bumper;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    extendEncoder = m_extend.extend_encoder();
    // SmartDashboard.putNumber("Extend Encoder", extendEncoder);
    
    switch(button) 
    {
      case Constants.A_Button:   ////////////////////////////////////////////////////////////
        if(right_bumper)    
        {
          extendTarget = Constants.EXTEND_DEFAULT;
        }
        else if(left_bumper)
        {
          extendTarget = Constants.EXTEND_DEFAULT;
        }
        else
        {
          extendTarget = Constants.EXTEND_SCORE_MIDDLE;
        }
        break;

      case Constants.B_Button:  ////////////////////////////////////////////////////////////
        if(right_bumper)
        {
          extendTarget = Constants.EXTEND_DEFAULT;
        }
        else if(left_bumper)
        {
          extendTarget = Constants.EXTEND_DEFAULT;
        }
        else
        {
          extendTarget = Constants.EXTEND_SCORE_HIGH;
        }
          break;

      case Constants.X_Button:    ////////////////////////////////////////////////////////////
        if(right_bumper)
        {
          extendTarget = Constants.EXTEND_DEFAULT;
        }
        else if(left_bumper)
        {
          extendTarget = Constants.EXTEND_DEFAULT;
        }
        else
        {
          extendTarget = Constants.EXTEND_DEFAULT;
        }
        break;

      case Constants.Y_Button:       ////////////////////////////////////////////////////////////
        if(right_bumper || left_bumper)
        {
          extendTarget = Constants.EXTEND_DEFAULT;
        }
        else
        {
        extendTarget = Constants.EXTEND_DEFAULT;
        }
        break;

      default:   ////////////////////////////////////////////////////////////
      extendTarget = Constants.EXTEND_DEFAULT;
    }

    m_extend.Extend_Goto_Length(extendTarget);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_extend.extend_off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}