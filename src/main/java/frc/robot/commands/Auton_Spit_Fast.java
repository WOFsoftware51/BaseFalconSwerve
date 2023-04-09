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


public class Auton_Spit_Fast extends CommandBase 
{

  private final Intake m_intake;
  private int init_counter = 0; 
  private int wait_length = 0;
  private int count = 0;
  private boolean endcommand = false;
  /** Creates a new Intake. */
  // private final CANdle m_candle;
  

  /** Creates a new Intake. */
  public Auton_Spit_Fast(Intake intake, int wait) 
  {
    this.m_intake = intake;
    addRequirements(intake);
    this.wait_length = wait;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_intake.intake_init();
    Global_Variables.have_game_piece = false;
    init_counter = 0; 
    count = 0;
    endcommand = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    m_intake.Intake_Reverse_Fast();

    if(init_counter < wait_length)
    {
      init_counter++;
    }
    else
    {
      endcommand = true;
    }
        

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_intake.Intake_Slow();
    Global_Variables.have_game_piece = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return endcommand;
  }
}
