// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.subsystems.Arm;


public class Arm_Setposition extends CommandBase 
{

  private final Arm m_arm;
  
  private double armEncoder = 0.0;
  private double armCANCoder = 0.0;
  private double armSpeed = 0.0;

  private int button = 0;
  private double robot_direction;
  private double robot_directionY;
  private boolean left_bumper = false;
  private boolean right_bumper = false;
  private int count = 0;
  private final double offset = -4.0;
  private int color = 0;
 
  private double armTarget;

  /** Creates a new Arm. */
  public Arm_Setposition(Arm arm, int Button) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = arm;
    addRequirements(arm);
    this.button = Button;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_arm.arm_init();
    left_bumper = Global_Variables.left_bumper;
    right_bumper = Global_Variables.right_bumper;
    robot_direction = Global_Variables.robot_direction;
    robot_directionY = Global_Variables.robot_directionY;
    color = m_arm.getAllianceColor();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    
    armEncoder = m_arm.arm_encoder();
    armCANCoder = m_arm.arm_CANCoder();
    armSpeed = m_arm.Arm_Speed();


    SmartDashboard.putNumber("Arm Speed", Conversions.falconToRPM(armSpeed, Constants.ARM_GEAR_RATIO));
    SmartDashboard.putNumber("Arm Encoder", armEncoder);
    SmartDashboard.putNumber("Arm CANCoder", armCANCoder);
    SmartDashboard.putNumber("Wrist Target", armTarget);

    if(count > 2 && (armEncoder < -20 || armEncoder > 20))
    {
      m_arm.updateEncoder();
      count = 0;
    }
    else
    {
      count++;
    }
    
    switch(button) 
    {
      case Constants.A_Button:   ////////////////////////////////////////////////////////////
        if(right_bumper)    
        {
          armTarget = Constants.ARM_PICKUP_CONE_DOWN*robot_direction;
        }
        else if(left_bumper)
        {
          armTarget = -(Constants.ARM_PICKUP_CONE_DOWN)*robot_direction;
        }
        else
        {
          armTarget = Constants.ARM_SCORE_MIDDLE*robot_direction;
        }
        break;

      case Constants.B_Button:  ////////////////////////////////////////////////////////////
      if(right_bumper)
      {
        armTarget = Constants.ARM_PICKUP_CONE*robot_direction;
      }
      else if(left_bumper)
      {
        armTarget = -(Constants.ARM_PICKUP_CONE)*robot_direction;
      }
      else
      {
        armTarget = Constants.ARM_SCORE_HIGH*robot_direction;
      }
        break;

      case Constants.X_Button:    ////////////////////////////////////////////////////////////
      if(right_bumper)
      {
        armTarget = Constants.ARM_PICKUP_CUBE*robot_direction;
      }
      else if(left_bumper)
      {
        armTarget = -(Constants.ARM_PICKUP_CUBE)*robot_direction;
      }
      else
      {
        armTarget = Constants.ARM_DEFAULT;
      }
        break;

      case Constants.Y_Button:       ////////////////////////////////////////////////////////////
      
      if(right_bumper)
      {
        armTarget = -(Constants.ARM_HUMAN_PLAYER_LOAD)*Global_Variables.robot_directionY*color;
      }
      else if(left_bumper)
      {
        armTarget = -(Constants.ARM_SHELF_HUMAN_PLAYER_LOAD)*Global_Variables.robot_directionY*color;
      }
      else
      {
        armTarget = Constants.ARM_DEFAULT;
      }
        break;

      default:   ////////////////////////////////////////////////////////////
      armTarget = Constants.ARM_DEFAULT;
    }
    if(armTarget > 20)
    {
      armTarget = armTarget + offset;
    }

    m_arm.Arm_Goto_Angle(armTarget);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_arm.arm_off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}