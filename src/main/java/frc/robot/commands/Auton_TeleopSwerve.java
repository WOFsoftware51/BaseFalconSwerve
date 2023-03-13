package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class Auton_TeleopSwerve extends CommandBase 
{    
    private Swerve s_Swerve;    
    private double translation;
    private double strafe;
    private double rotation;
    private double turn_error;
    private double rotationPercent;
    private final double m_distance;
    private int counter = 0;
    private  boolean endCommand = false;

    public Auton_TeleopSwerve(Swerve s_Swerve, double translation, double strafe, double rotation, double distance) 
    {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translation = translation;
        this.strafe = strafe;
        this.rotation = rotation;
        this.m_distance = distance;

    }

    @Override
    public void initialize() 
    {
      endCommand = false;
      counter = 0;
      s_Swerve.resetModulesToAbsolute();
      
    }

    @Override
    public void execute() 
    {  
      /* 
        if(time>counter)
        { 
            s_Swerve.drive(
                new Translation2d(translation, strafe).times(Constants.Swerve.maxSpeed), 
                rotation * Constants.Swerve.maxAngularVelocity*s_Swerve.SpeedModifier, 
                false, 
                true
            );
            counter++;
        }
        else
        {
            endCommand = true;
        }
*/
        SmartDashboard.putNumber("Distance",s_Swerve.Distance);
        SmartDashboard.putNumber("Yaw",s_Swerve.yawFixed);
        SmartDashboard.putNumber("turn_error",turn_error);


        turn_error = s_Swerve.yawFixed - rotation;
 
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
        

        if(s_Swerve.Distance > m_distance)
        {
          translation = 0;
          strafe = 0;
          rotation = 0;
          endCommand = true;
        }
        else
        {
        s_Swerve.drive(
          new Translation2d(translation, strafe).times(Constants.Swerve.maxSpeed), 
          rotation * Constants.Swerve.maxAngularVelocity*s_Swerve.SpeedModifier, 
          false, 
          true
          );
        }
    
      
      

    }

    public void end(boolean interrupted) 
    {
      
       // s_Swerve.resetModulesToAbsolute();
      s_Swerve.drive(
        new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
        0 * Constants.Swerve.maxAngularVelocity, 
        false, 
        true
      ); 
    }

    @Override
    public boolean isFinished() 
    { 
      return endCommand;

    }  
}