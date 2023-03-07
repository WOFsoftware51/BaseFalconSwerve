package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift_Pneumatic;
import edu.wpi.first.wpilibj2.command.CommandBase;

//import java.util.function.Double;

public class Auton_Lift extends CommandBase 
{
  
    //private final Drive_Auton drivetrain;
    private int init_counter = 0; 
    private int wait_length = 0;
    Lift_Pneumatic m_lift;


    

    public Auton_Lift(int wait,Lift_Pneumatic lift)
      {
        this.wait_length = wait;
        addRequirements(lift);
        this.m_lift = lift;
      }

      @Override
      public void initialize() 
      {
        init_counter = 0; 
        m_lift.Arm_Pnematics_On();
      }

    @Override
    public void execute() 
    {
      init_counter++; 
        //rotationPercent = 0;
    }
       


    @Override
    public void end(boolean interrupted) 
    {
      init_counter = 0; 
        // Stop the drivetrain
      m_lift.Arm_Pnematics_Off();
    }

    @Override
    public boolean isFinished() 
    { 
      if(init_counter>=wait_length) 
      { 
        return true;
      } 
        else 
      {
       return false;
      }
    }
}