// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class Auton_Score extends CommandBase 
{

  private final Arm m_arm;
  private final Wrist m_wrist;
  private double armEncoder = 0.0;
  private double armCANCoder = 0.0;
  private double wristEncoder = 0.0;
  private double wristCANCoder = 0.0;
  private double armSpeed = 0.0;
  private double wristSpeed = 0.0;

  private double armTarget;
  private double wristTarget;


  /** Creates a new Arm. */
  public Auton_Score(Arm arm, double armAngle, Wrist wrist, double wristAngle) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = arm;
    addRequirements(arm);
    this.m_wrist = wrist;
    addRequirements(wrist);
    armTarget = armAngle;
    wristTarget = wristAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_arm.arm_init();
    m_wrist.wrist_init();
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

    SmartDashboard.putNumber("Wrist Speed", Conversions.falconToRPM(wristSpeed, Constants.WRIST_GEAR_RATIO));
    SmartDashboard.putNumber("Arm Speed", Conversions.falconToRPM(armSpeed, Constants.ARM_GEAR_RATIO));
    SmartDashboard.putNumber("wrist Encoder", wristEncoder);
    SmartDashboard.putNumber("wrist CANCoder", wristCANCoder);
    SmartDashboard.putNumber("Arm Encoder", armEncoder);
    SmartDashboard.putNumber("Arm CANCoder", armCANCoder);
    SmartDashboard.putNumber("Arm Target", wristTarget);
    SmartDashboard.putNumber("Wrist Target", armTarget);
    m_wrist.Wrist_Goto_Angle(wristTarget);
    m_arm.Arm_Goto_Angle(armTarget);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_arm.arm_off();
    m_wrist.Wrist_Off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
