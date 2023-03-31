// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.subsystems.Wrist;

public class Wrist_Setposition extends CommandBase 
{

  private final Wrist m_wrist;
  
  
  private double wristEncoder = 0.0;
  private double wristCANCoder = 0.0;
  private double wristSpeed = 0.0;
  private int button = 0;
  private double robot_direction;
  private double robot_directionY;
  private boolean left_bumper = false;
  private boolean right_bumper = false;
  private int count = 0;
  private int offset = 2;
  private int color = 0;
  private double wristTarget;


  /** Creates a new Arm. */
  public Wrist_Setposition(Wrist wrist, int Button) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_wrist = wrist;
    addRequirements(wrist);
    this.button = Button;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_wrist.wrist_init();
    left_bumper = Global_Variables.left_bumper;
    right_bumper = Global_Variables.right_bumper;
    robot_direction = Global_Variables.robot_direction;
    robot_directionY = Global_Variables.robot_directionY;
    color = m_wrist.getAllianceColor();

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    wristEncoder = m_wrist.wrist_encoder();
    wristCANCoder = m_wrist.wrist_CANCoder();
    wristSpeed = m_wrist.wrist_Speed();

    SmartDashboard.putNumber("Wrist Speed", Conversions.falconToRPM(wristSpeed, Constants.WRIST_GEAR_RATIO));
    SmartDashboard.putNumber("wrist Encoder", wristEncoder);
    SmartDashboard.putNumber("wrist CANCoder", wristCANCoder);
    SmartDashboard.putNumber("Arm Target", wristTarget);

    if(count > 2 && button != Constants.Y_Button)
    {
      m_wrist.updateEncoder();
      count = 0;
    }
    else
    {
      count++;
    }
    
    switch(button) {
      case Constants.A_Button:   ////////////////////////////////////////////////////////////
        if(right_bumper)    
        {
          wristTarget = Constants.WRIST_PICKUP_CONE_DOWN*robot_direction;
        }
        else if(left_bumper)
        {
          wristTarget = Constants.WRIST_PICKUP_CONE_DOWN*-robot_direction;
        }
        else
        {
          wristTarget = Constants.WRIST_SCORE*robot_direction;
        }
        break;

      case Constants.B_Button:  ////////////////////////////////////////////////////////////
      if(right_bumper)
      {
        wristTarget = Constants.WRIST_PICKUP_CONE*robot_direction;
      }
      else if(left_bumper)
      {
        wristTarget = Constants.WRIST_PICKUP_CONE*-robot_direction;
      }
      else
      {
        wristTarget = Constants.WRIST_SCORE*robot_direction;
      }
        break;

      case Constants.X_Button:    ////////////////////////////////////////////////////////////
      if(right_bumper)
      {
        wristTarget = Constants.WRIST_PICKUP_CUBE*robot_direction;
      }
      else if(left_bumper)
      {
        wristTarget = Constants.WRIST_PICKUP_CUBE*-robot_direction;
      }
      else
      {
        wristTarget = Constants.WRIST_SCORE_LOW*robot_direction;
      }
        break;

      case Constants.Y_Button:       ////////////////////////////////////////////////////////////
     
      if(right_bumper || left_bumper)
      {
        wristTarget = (Constants.WRIST_HUMAN_PLAYER_LOAD)*robot_directionY*color;
      }
      else
      {
        wristTarget = Constants.WRIST_DEFAULT;
      }

      break;

      default:   ////////////////////////////////////////////////////////////
      wristTarget = Constants.WRIST_DEFAULT;
    }

    if(wristTarget < 5)
    {
      wristTarget = wristTarget + offset;
    }

    m_wrist.Wrist_Goto_Angle(wristTarget);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_wrist.Wrist_Off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}