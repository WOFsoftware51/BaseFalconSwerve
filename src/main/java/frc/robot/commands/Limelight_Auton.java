package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class Limelight_Auton extends CommandBase 
{    
    private Swerve s_Swerve;    
    private double rotation;
    private double turn_error;
    private double rotationPercent;
    private int counter = 0;
    private int m_timer = 0;
    private  boolean endCommand = false;

    public Limelight_Auton(Swerve s_Swerve, double rotation, int timer) 
    {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.rotation = rotation;
        this.m_timer = timer;

    }

    @Override
    public void initialize() 
    {
      endCommand = false;
      counter = 0;
      s_Swerve.resetDrive();
    }

    @Override
    public void execute() 
    {  
        SmartDashboard.putNumber("Yaw",s_Swerve.yawFixed);
        SmartDashboard.putNumber("turn_error",turn_error);


      turn_error = rotation - s_Swerve.yaw();
 
      if(turn_error<-30)
      {
        rotationPercent = 0.3;
      }
        else if(turn_error<-10)
      {
        rotationPercent = 0.15;
      }
        else if(turn_error<-1)
      {
        rotationPercent = 0.05;
      }
        else if(turn_error>30)
      {
        rotationPercent = -0.3;
      }
        else if(turn_error>10)
      {
        rotationPercent = -0.15;
      }
        else if(turn_error>1)
      {
        rotationPercent = -0.05;
      }
        else
      {
        rotationPercent = 0.0;
      }
      

        s_Swerve.drive(
        new Translation2d(-0.05*s_Swerve.ty, -0.015*s_Swerve.tx).times(Constants.Swerve.maxSpeed), 
        rotationPercent * Constants.Swerve.maxAngularVelocity, 
        true, 
        true
        );
        counter++;

      if(counter>m_timer*20)
      {
        endCommand = true;
      }
      if(-1<s_Swerve.tx && s_Swerve.tx<1 && s_Swerve.ty<2 && s_Swerve.ty>-2)
      {
        endCommand = true;
      }

    }

    public void end(boolean interrupted) 
    {
      s_Swerve.resetDrive();
      
      s_Swerve.drive(
        new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
        0 * Constants.Swerve.maxAngularVelocity, 
        true,
        true
      ); 
    }

    @Override
    public boolean isFinished() 
    { 
      return endCommand;

    }  
}