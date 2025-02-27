package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants 
{

    public static final double EXTEND_OFFSET = -4555.55;

    public static final double AUTON_ARM_OFFSET = -5;




    public static final double stickDeadband = 0.1;
    public static final String CANIVORE_NAME = "CANivore";
    public static final double DRIVE_SPEED = 0.6;
    public static final double ARM_GEAR_RATIO = 268.6815;   //  22.67*(64/18)*(60/15)
    public static final double WRIST_GEAR_RATIO =  145.833333;   // 100*(36/22) //164.63636
    public static final double ARM_CONVERSION = 2048*ARM_GEAR_RATIO/360; // Degrees*ARM_CONVERSION = Encoder value
    public static final double WRIST_CONVERSION = 2048*WRIST_GEAR_RATIO/360; // Degrees*ARM_CONVERSION = Encoder value

    public static final double ARM_DEFAULT = 0;
    public static final double WRIST_DEFAULT = 0;
    public static final double EXTEND_DEFAULT = 0; 
    public static final double WRIST_SCORE = 82;
    public static final double WRIST_SCORE_LOW = 90;
    public static final double WRIST_SCORE_HIGH_AUTON = 90;
    public static final double WRIST_SCORE_FAST_CUBE = 54;
    public static final double WRIST_PICKUP_CUBE = -18;
    public static final double WRIST_PICKUP_CONE = -19;
    public static final double WRIST_PICKUP_CONE_DOWN = 4.938085;
    public static final double ARM_SCORE_HIGH = 45.5;
    public static final double ARM_SCORE_MIDDLE = 46.5;
    public static final double ARM_PICKUP_CUBE = -105;
    public static final double ARM_PICKUP_CONE = -99;
    public static final double ARM_PICKUP_CONE_DOWN = -111.614931;

    public static final double EXTEND_SCORE_HIGH = -82000;

    public static final double EXTEND_SCORE_MIDDLE = -40000;
    public static final double ARM_HUMAN_PLAYER_LOAD = 91;
    public static final double ARM_SHELF_HUMAN_PLAYER_LOAD = 58;
    public static final double WRIST_HUMAN_PLAYER_LOAD = -65;


    
    public static final double ARM_OFFSET = 182;
    public static final double WRIST_OFFSET = 152.671741;

    public static final double Mod0_ROTATION_OFFSET = 31.376953;
    public static final double Mod1_ROTATION_OFFSET = 197.314453;
    public static final double Mod2_ROTATION_OFFSET = 205.048828;
    public static final double Mod3_ROTATION_OFFSET = 258.046875;





    public static final class Swerve 
    {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
        
        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(19.5); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(19.5); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;
        
        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.667 / 12); //TODO: This must be tuned to specific robot  //0.32 is origninal value
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot 
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
          
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 20; // 22
            public static final int angleMotorID = 30; //32
            public static final int canCoderID = 10; //10
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(Mod0_ROTATION_OFFSET); //210.36
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 21; //20
            public static final int angleMotorID = 31; // 30
            public static final int canCoderID = 11; //11
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(Mod1_ROTATION_OFFSET);//133.8
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 22; //23
            public static final int angleMotorID = 32; // 33
            public static final int canCoderID = 12; //12
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(Mod2_ROTATION_OFFSET);//13.27
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 
        { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 23; //21
            public static final int angleMotorID = 33; //31
            public static final int canCoderID = 13; //13
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(Mod3_ROTATION_OFFSET); //357.4
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 4.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = 10; //Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        public static final double AUTON_ARM_SPEED = 0.5;

        public static final double kBalanceSpeedMetersPerSecond = 4.5/2;
        public static final double kBalanceAccelerationMetersPerSecondSquared = 3/2;

        public static final double kPXController = 1.6; //20;  //1.3
        public static final double kPYController = 1.3;  //5; //1.3
        public static final double kPThetaController = -10;//-20  //-10;;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    public static final class Intake_
    {
        public static final int Intake_Motor = 44; //41
        public static final int Wrist_Motor = 42;
    }
    public static final int Extend_Motor = 43;
    public static final int Arm_Motor_Slave = 46;
    public static final int Arm_Motor = 45;
    public static final int Arm_CANCoder = 40;
    public static final int Wrist_CANCoder = 51;
    public static final int CANdleID = 0;

    public static final int A_Button = 1;
    public static final int B_Button = 2;
    public static final int X_Button = 3;
    public static final int Y_Button = 4;


}
