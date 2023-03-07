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
import frc.robot.subsystems.Extend;
import frc.robot.subsystems.Wrist;

public class Arm_Wrist_Extend_Setposition extends CommandBase 
{

  private final Arm m_arm;
  private final Wrist m_wrist;
  private final Extend m_extend;

  
  private double armEncoder = 0.0;
  private double armCANCoder = 0.0;
  private double wristEncoder = 0.0;
  private double wristCANCoder = 0.0;
  private double armSpeed = 0.0;
  private double wristSpeed = 0.0;
  private double extendEncoder = 0.0;
  private int button = 0;
  private double robot_direction;
  private boolean left_bumper = false;
  private boolean right_bumper = false;
 


  private double armTarget;
  private double wristTarget;
  private double extendTarget;


  /** Creates a new Arm. */
  public Arm_Wrist_Extend_Setposition(Arm arm,  Wrist wrist, Extend extend, int Button) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = arm;
    addRequirements(arm);
    this.m_wrist = wrist;
    addRequirements(wrist);
    this.m_extend = extend;
    addRequirements(extend);
    this.button = Button;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_arm.arm_init();
    m_wrist.wrist_init();
    m_extend.extend_init();
    left_bumper = Global_Variables.left_bumper;
    right_bumper = Global_Variables.right_bumper;
    robot_direction = Global_Variables.robot_direction;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    
    armEncoder = m_arm.arm_encoder();
    armCANCoder = m_arm.arm_CANCoder();
    wristEncoder = m_wrist.wrist_encoder();
    wristCANCoder = m_wrist.wrist_CANCoder();
    armSpeed = m_arm.Arm_Speed();
    wristSpeed = m_wrist.wrist_Speed();
    extendEncoder = m_extend.extend_encoder();

    SmartDashboard.putNumber("Wrist Speed", Conversions.falconToRPM(wristSpeed, Constants.WRIST_GEAR_RATIO));
    SmartDashboard.putNumber("Arm Speed", Conversions.falconToRPM(armSpeed, Constants.ARM_GEAR_RATIO));
    SmartDashboard.putNumber("wrist Encoder", wristEncoder);
    SmartDashboard.putNumber("wrist CANCoder", wristCANCoder);
    SmartDashboard.putNumber("Arm Encoder", armEncoder);
    SmartDashboard.putNumber("Extend Encoder", extendEncoder);
    SmartDashboard.putNumber("Arm CANCoder", armCANCoder);
    SmartDashboard.putNumber("Arm Target", wristTarget);
    SmartDashboard.putNumber("Wrist Target", armTarget);
    
    switch(button) {
      case Constants.A_Button:   ////////////////////////////////////////////////////////////
        if(right_bumper)    
        {
          wristTarget = Constants.WRIST_PICKUP_CONE_DOWN*robot_direction;
          armTarget = Constants.ARM_PICKUP_CONE_DOWN*robot_direction;
          extendTarget = Constants.EXTEND_DEFAULT;
        }
        else if(left_bumper)
        {
          wristTarget = Constants.WRIST_PICKUP_CONE_DOWN*-robot_direction;
          armTarget = Constants.ARM_PICKUP_CONE_DOWN*-robot_direction;
          extendTarget = Constants.EXTEND_DEFAULT;
        }
        else
        {
          wristTarget = Constants.WRIST_SCORE*robot_direction;
          armTarget = Constants.ARM_SCORE_MIDDLE*robot_direction;
          extendTarget = Constants.EXTEND_SCORE_MIDDLE;
        }
        break;

      case Constants.B_Button:  ////////////////////////////////////////////////////////////
      if(right_bumper)
      {
        wristTarget = Constants.WRIST_PICKUP_CONE*robot_direction;
        armTarget = Constants.ARM_PICKUP_CONE*robot_direction;
        extendTarget = Constants.EXTEND_DEFAULT;
      }
      else if(left_bumper)
      {
        wristTarget = Constants.WRIST_PICKUP_CONE*-robot_direction;
        armTarget = Constants.ARM_PICKUP_CONE*-robot_direction;
        extendTarget = Constants.EXTEND_DEFAULT;
      }
      else
      {
        wristTarget = Constants.WRIST_SCORE*robot_direction;
        armTarget = Constants.ARM_SCORE_HIGH*robot_direction;
        extendTarget = Constants.EXTEND_SCORE_HIGH;
      }
        break;

      case Constants.X_Button:    ////////////////////////////////////////////////////////////
      if(right_bumper)
      {
        wristTarget = Constants.WRIST_PICKUP_CUBE*robot_direction;
        armTarget = Constants.ARM_PICKUP_CUBE*robot_direction;
        extendTarget = Constants.EXTEND_DEFAULT;
      }
      else if(left_bumper)
      {
        wristTarget = Constants.WRIST_PICKUP_CUBE*-robot_direction;
        armTarget = Constants.ARM_PICKUP_CUBE*-robot_direction;
        extendTarget = Constants.EXTEND_DEFAULT;
      }
      else
      {
        wristTarget = Constants.WRIST_SCORE_LOW*robot_direction;
        armTarget = Constants.ARM_DEFAULT;
        extendTarget = Constants.EXTEND_DEFAULT;
      }
        break;

        case Constants.Y_Button:       ////////////////////////////////////////////////////////////
        wristTarget = Constants.WRIST_DEFAULT;
        armTarget = Constants.ARM_DEFAULT;
        extendTarget = Constants.EXTEND_DEFAULT;
        break;

      default:   ////////////////////////////////////////////////////////////
      wristTarget = Constants.WRIST_DEFAULT;
      armTarget = Constants.ARM_DEFAULT;
      extendTarget = Constants.EXTEND_DEFAULT;
    }

    m_wrist.Wrist_Goto_Angle(wristTarget);
    m_arm.Arm_Goto_Angle(armTarget);
    m_extend.Extend_Goto_Length(extendTarget);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_arm.arm_off();
    m_wrist.Wrist_Off();
    m_extend.extend_off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}