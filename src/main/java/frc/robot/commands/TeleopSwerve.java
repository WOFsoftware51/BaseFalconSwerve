package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Dynamic;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase 
{    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    public double speedModifier = Constants.DRIVE_SPEED;


    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void initialize()
    {
        speedModifier = Constants.DRIVE_SPEED;
    }
    
    @Override
    public void execute() 
    {
        if(Global_Variables.left_bumper_boost)
        {
            speedModifier = 0.1;   
        }
        else if(Global_Variables.right_bumper_boost)
        {
            speedModifier = 1.0;
        }
        else
        {
            speedModifier = Constants.DRIVE_SPEED;
        }



                /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

       
        double strafe = strafeVal*speedModifier;
        double translation = translationVal*speedModifier;

        SmartDashboard.putNumber("Forward Angle", s_Swerve.getPitch());
        SmartDashboard.putNumber("Side Angle", s_Swerve.getRoll());

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translation, strafe).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity*speedModifier, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
        SmartDashboard.putNumber("X", translation*100);
        SmartDashboard.putNumber("Y", strafe*100);

        SmartDashboard.putBoolean("Left Trigger", Global_Variables.left_bumper_boost);
        SmartDashboard.putBoolean("Right Trigger", Global_Variables.right_bumper_boost);

    }
}