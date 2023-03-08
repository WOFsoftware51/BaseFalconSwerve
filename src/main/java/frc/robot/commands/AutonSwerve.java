package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Dynamic;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutonSwerve extends CommandBase 
{    
    private Swerve s_Swerve;    
    private double xspeed = 0.0;
    private double yspeed = 0.0;
    private double xRotation;
    private int count = 0;
    private int time;
    private boolean endCommand = false;


    public AutonSwerve(Swerve s_Swerve, double ySpeed, double xSpeed, double xRotation, int time) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.xRotation = xRotation;
        this.yspeed = ySpeed;
        this.xspeed = xSpeed;
        this.time = time;
    }

    @Override
    public void initialize()
    {
        endCommand = false;
        count = 0;
    }
    
    @Override
    public void execute() 
    {
        /* Get Values, Deadband*/
       
         xspeed = xspeed *s_Swerve.SpeedModifier;
         yspeed = yspeed *s_Swerve.SpeedModifier;

        SmartDashboard.putNumber("Forward Angle", s_Swerve.getPitch());
        SmartDashboard.putNumber("Side Angle", s_Swerve.getRoll());

        /* Drive */
        s_Swerve.drive(
            new Translation2d(yspeed, xspeed).times(Constants.Swerve.maxSpeed), 
            xRotation * Constants.Swerve.maxAngularVelocity*s_Swerve.SpeedModifier, 
            false, 
            true
        );
        count++;
        if (count > time)
        {
            count = 0;
            endCommand = true;
        }

    }
    @Override
    public boolean isFinished() 
    { 
      return endCommand;

    }  
}
