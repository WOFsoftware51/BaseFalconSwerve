package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Global_Variables;
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
    private double translationVal = 0;
    private double strafeVal = 0;
    private double turn_error;
    private double rotationPercent;
    private final double m_distance;
    private int counter = 0;
    private int m_timer = 0;
    private  boolean endCommand = false;
    private boolean m_timerOn = false;

    public Auton_TeleopSwerve(Swerve s_Swerve, double translation, double strafe, double rotation, double distance, int timer, boolean timerOn) 
    {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translation = translation;
        this.strafe = strafe;
        this.rotation = rotation;
        this.m_distance = distance;
        this.m_timer = timer;
        this.m_timerOn = timerOn;

    }

    @Override
    public void initialize() 
    {
      endCommand = false;
      counter = 0;
      s_Swerve.Distance = 0;
      s_Swerve.resetDrive();
    }

    @Override
    public void execute() 
    {  
        SmartDashboard.putNumber("Distance",s_Swerve.Distance);
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
        
      if(m_timerOn)
      {
        if(s_Swerve.Distance > m_distance && counter > m_timer)
        {
          translationVal = 0;
          strafeVal = 0;
          rotationPercent = 0;
          endCommand = true;
        }
        else if(s_Swerve.Distance > 0.01 + m_distance)
        {
          translationVal = translation * 0.5;
          strafeVal = strafe * 0.5;
        } 
        else
        {
          translationVal = translation;
          strafeVal = strafe;
        }
        counter++;
      }
      else
      {
        if(s_Swerve.Distance > m_distance)
        {
          translationVal = 0;
          strafeVal = 0;
          rotationPercent = 0;
          endCommand = true;
        }
        else if(s_Swerve.Distance > 0.01 + m_distance)
        {
          translationVal = translation * 0.5;
          strafeVal = strafe * 0.5;
        } 
        else
        {
          translationVal = translation;
          strafeVal = strafe;
        }
      }

        s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
        rotationPercent * Constants.Swerve.maxAngularVelocity, 
        true, 
        true
        );
    }

    public void end(boolean interrupted) 
    {
      s_Swerve.resetDrive();
      s_Swerve.Distance = 0;
      
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