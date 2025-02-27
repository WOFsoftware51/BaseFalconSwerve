package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.Global_Variables;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Swerve extends SubsystemBase 
{
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public double SpeedModifier = Constants.DRIVE_SPEED;
    public Pigeon2 gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.CANIVORE_NAME);
    public double yawFixed = 0.0;
    public double Distance = 0.0;
    public double speed = 0.0;

    public double tx = 0.0;
    public double ty = 0.0;
    public double tv = 0.0;
    public double botpose[];

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private double cameraCount = 0;

    public Swerve()
    {
        gyro.configFactoryDefault();
        zeroGyro();
        
        mSwerveMods = new SwerveModule[]
        {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
        
        
        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }
    /* The fromFieldRelativeSpeeds method can be simplified to 
    
       fromFieldRelativeSpeeds()
        {
            sin(b+a) -> x component
           -cos(b+a) -> y component
        }

        Where b is the joystick angle and a is the angle of the robot
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
     {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates
            (
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds
                                (
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods)
        {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) 
    {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void Boost_On()
    {
        SpeedModifier = 1.0;
    }

    public void Boost_Off() 
      {
        SpeedModifier = Constants.DRIVE_SPEED;
      }

    public void AntiBoost_On() 
    {
    SpeedModifier = 0.1;
    }

    public double getPitch()
    {
    return gyro.getPitch();
    }

    public double getRoll()
    {
    return gyro.getRoll();
    }

    public double yaw()
    {
    return gyro.getYaw();
    }
    
    public void resetDrive() 
    {
        for(SwerveModule mod : mSwerveMods)
        {
            mod.resetDrPosition();
        }
    }

    public void music_init()
    {
        for(SwerveModule mod : mSwerveMods)
        {
            mod.add_instruments();
        }
        SwerveModule.music_init();  
    }

    public void play_music()
    {
        SwerveModule.play_music();
    }

    public void stop_music()
    {
        SwerveModule.stop_music();
    }

    public void addToInstruments(TalonFX[] motor)
    {
        SwerveModule.addToInstruments(motor);
    }

    public void cameraPipeline()
    {
        cameraCount++;
        table.getEntry("pipeline").setNumber(cameraCount% 2);
    }

   
 // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
         new InstantCommand(() -> 
         {
           // Reset odometry for the first path you run during auto
           if(isFirstPath){
               this.resetOdometry(traj.getInitialHolonomicPose());
           }
         }),
         new PPSwerveControllerCommand(
             traj, 
             this::getPose, // Pose supplier
             Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
             new PIDController(Constants.AutoConstants.kPXController, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             new PIDController(Constants.AutoConstants.kPYController, 0, 0), // Y controller (usually the same values as X controller)
             new PIDController(Constants.AutoConstants.kPThetaController, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             this::setModuleStates, // Module states consumer
             false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
             this // Requires this drive subsystem
         )
     );
    }
 

    @Override
    public void periodic()
    {

        tv = table.getEntry("tv").getDouble(0);
        ty = table.getEntry("ty").getDouble(0);
        tx = table.getEntry("tx").getDouble(0);
        botpose = table.getEntry("botpose").getDoubleArray(new double[6]); 
        


        swerveOdometry.update(getYaw(), getModulePositions());  
        yawFixed = Math.abs(gyro.getYaw()% 360);

        if(yawFixed < 270 && yawFixed > 90)
        {
          Global_Variables.robot_direction = -1.0;
        }
        else
        {
         Global_Variables.robot_direction = 1.0;
        }

        if(yawFixed < 180 && yawFixed > 0)
        {
          Global_Variables.robot_directionY = -1.0;
        }
        else
        {
         Global_Variables.robot_directionY = 1.0;
        }

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Drive Encoder" + mod.moduleNumber, mod.getPosition().distanceMeters);
        }
        
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
        SmartDashboard.putNumber("Yaw Fixed", yawFixed);
        SmartDashboard.putNumber("Roll", getRoll());
        SmartDashboard.putNumber("Pitch", getPitch());
 
        SmartDashboard.putNumber("tx", tx);
        SmartDashboard.putNumber("tv", tv);
        SmartDashboard.putNumber("ty", ty);

        SmartDashboard.putNumber("Robot Forward", Global_Variables.robot_direction);
      //  SmartDashboard.putNumber("Robot Forward", Global_Variables.robot_direction);

        Distance = (Math.abs(mSwerveMods[0].getPosition().distanceMeters)+Math.abs(mSwerveMods[1].getPosition().distanceMeters)+Math.abs(mSwerveMods[2].getPosition().distanceMeters)+Math.abs(mSwerveMods[3].getPosition().distanceMeters))/4.0;
    }
}