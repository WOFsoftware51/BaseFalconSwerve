package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class Auton_Balance_TeleopSwerve extends CommandBase 
{    
    private Swerve s_Swerve;    
    private double translation;
    private double strafe;
    private double rotation;
    private double pitch;
    public Auton_Balance_TeleopSwerve(Swerve s_Swerve, double translation, double strafe, double rotation) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translation = translation;
        this.strafe = strafe;
        this.rotation = rotation;
    }

    @Override
    public void initialize() 
    {
      pitch = s_Swerve.getPitch();
    }

    @Override
    public void execute() 
    {
      pitch = s_Swerve.getPitch();
      if(pitch > 5)
      {
        /* Drive */
        s_Swerve.drive(
          new Translation2d(-translation, strafe).times(Constants.Swerve.maxSpeed), 
          rotation * Constants.Swerve.maxAngularVelocity, 
          false, 
          true
        );
      }
      else if(pitch < -5)
      {
         /* Drive */
        s_Swerve.drive(
          new Translation2d(translation, strafe).times(Constants.Swerve.maxSpeed), 
          rotation * Constants.Swerve.maxAngularVelocity, 
          false, 
          true
        );
      }
      else if(pitch < 5 && pitch < -5)
      {
        s_Swerve.drive(
          new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
          0 * Constants.Swerve.maxAngularVelocity, 
          false, 
          true
        ); 
      }
      
    }

    @Override
    public boolean isFinished() 
    { 
    if(pitch < 5 && pitch < -5)
    {
      return true;
    }
    else
    {
      return false;
    }
}  
}