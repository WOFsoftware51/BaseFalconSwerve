// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Global_Variables;
import frc.robot.subsystems.CANdle_Subsystem;

public class CANdle_Intake extends CommandBase {
  /** Creates a new CANdle_Intake. */
  CANdle_Subsystem m_candle;
  private boolean end = false;
  private int count = 0;

  public CANdle_Intake(CANdle_Subsystem candle) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_candle = candle;
    addRequirements(candle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_candle.CANdle_init();
    end = false;
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
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
      m_candle.CANdle_Purple_Blink();
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
