package frc.robot;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 * 
 */
public class RobotContainer
{
    /* Controllers */
   // private final Joystick driver = new Joystick(0);
    private final XboxController m_controller = new XboxController(0);
    private final XboxController m_controller2 = new XboxController(1);
    private final SendableChooser<Integer> a_chooser = new SendableChooser<>();

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int intakeAxis = XboxController.Axis.kRightX.value;
  //  private final int robotCentric = XboxController.Button.kLeftBumper.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(m_controller2, XboxController.Button.kBack.value);
    private final JoystickButton robotCentric = new JoystickButton(m_controller2, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Intake m_intake = new Intake();
    private final CANdle_Subsystem m_candle = new CANdle_Subsystem();
    private final Lift_Pneumatic m_lift = new Lift_Pneumatic();
    private final Arm m_arm = new Arm();
    private final Auton_Subsystem m_auton = new Auton_Subsystem();
    private final Extend m_extend = new Extend();
    private final Wrist m_wrist = new Wrist();





    



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() 
    {

        SmartDashboard.putData("Auton", a_chooser);
    
        a_chooser.setDefaultOption("Red Two Piece Auto", 4);
        a_chooser.addOption("Example Auto", 1);
        a_chooser.addOption("Balance", 2);
        a_chooser.addOption("Blue Two Piece Auto", 3);
        a_chooser.addOption("One Piece Auto", 5);
        a_chooser.addOption("Sit", 6);
        a_chooser.addOption("Cone + Leave Zone", 7);

        s_Swerve.setDefaultCommand
        (
            new TeleopSwerve
            (
                s_Swerve, 
                () -> m_controller2.getRawAxis(translationAxis), 
                () -> m_controller2.getRawAxis(strafeAxis), 
                () -> m_controller2.getRawAxis(rotationAxis),
                () -> robotCentric.getAsBoolean()
            )
         );
     //  new Trigger(m_controller2::getAButton).whileTrue(new CANdle_Orange_Command(m_candle));  
     //  new Trigger(m_controller2::getBButton).whileTrue(new CANdle_Purple_Command(m_candle)); 

       m_candle.setDefaultCommand(new CANdle_Default(m_candle));
       m_arm.setDefaultCommand(new Arm_Command(m_arm, () -> modifyAxis(m_controller.getLeftY())));
       m_wrist.setDefaultCommand(new Wrist_Command (() -> modifyAxis(m_controller.getRightY()), m_wrist));
       new Trigger(()-> m_controller.getLeftTriggerAxis() > 0.80).whileTrue(new Intake_Command(m_intake));
       new Trigger(()-> m_controller.getLeftTriggerAxis() > 0.80).whileTrue(new CANdle_Intake(m_candle));
       new Trigger(()-> m_controller.getRightTriggerAxis() > 0.80).whileTrue(new Intake_Reverse_Command(m_intake));
      // new Trigger(m_controller::getAButton).whileTrue(new ScoreDefault(m_arm, m_intake, true, Constants.ARM_DEFAULT, Constants.WRIST_DEFAULT));
     

     /* old controls
       new Trigger(m_controller::getBButton).whileTrue(new ScoreMiddle(m_arm, Constants.ARM_SCORE_HIGH*Global_Variables.robot_direction, m_wrist, Constants.WRIST_SCORE*Global_Variables.robot_direction));      
       new Trigger(m_controller::getBButton).whileTrue(new Auton_Arm_Extend(m_extend, Constants.EXTEND_SCORE_HIGH));
       new Trigger(m_controller::getAButton).whileTrue(new ScoreMiddle(m_arm, Constants.ARM_PICKUP_CONE, m_wrist, Constants.WRIST_PICKUP_CONE));
       new Trigger(m_controller::getYButton).whileTrue(new ScoreMiddle(m_arm, 0.0, m_wrist, 0.0));
       new Trigger(m_controller::getYButton).whileTrue(new Auton_Arm_Extend(m_extend, 0.0));
       new Trigger(m_controller::getXButton).whileTrue(new ScoreMiddle(m_arm, Constants.ARM_PICKUP_CONE_DOWN, m_wrist, Constants.WRIST_PICKUP_CONE_DOWN));
       //new Trigger(m_controller::getBButton).whileTrue(new AutoScoreCone(m_arm, m_intake, m_extend, m_wrist));

        new Trigger(m_controller::getXButton).whileTrue(new Arm_Wrist_Extend_Setposition(m_arm, m_wrist, m_extend, Constants.X_Button));
        new Trigger(m_controller::getAButton).whileTrue(new Arm_Wrist_Extend_Setposition(m_arm, m_wrist, m_extend, Constants.A_Button));
        new Trigger(m_controller::getBButton).whileTrue(new Arm_Wrist_Extend_Setposition(m_arm, m_wrist, m_extend, Constants.B_Button));
        new Trigger(m_controller::getYButton).whileTrue(new Arm_Wrist_Extend_Setposition(m_arm, m_wrist, m_extend, Constants.Y_Button));
        */
        new Trigger(m_controller::getXButton).whileTrue(new Extend_Setposition(m_extend, Constants.X_Button));
        new Trigger(m_controller::getAButton).whileTrue(new Extend_Setposition(m_extend, Constants.A_Button));
        new Trigger(m_controller::getBButton).whileTrue(new Extend_Setposition(m_extend, Constants.B_Button));
        new Trigger(m_controller::getYButton).whileTrue(new Extend_Setposition(m_extend, Constants.Y_Button));
		new Trigger(m_controller::getXButton).whileTrue(new Arm_Setposition(m_arm, Constants.X_Button));
        new Trigger(m_controller::getAButton).whileTrue(new Arm_Setposition(m_arm, Constants.A_Button));
        new Trigger(m_controller::getBButton).whileTrue(new Arm_Setposition(m_arm, Constants.B_Button));
        new Trigger(m_controller::getYButton).whileTrue(new Arm_Setposition(m_arm, Constants.Y_Button));
		new Trigger(m_controller::getXButton).whileTrue(new Wrist_Setposition(m_wrist, Constants.X_Button));
        new Trigger(m_controller::getAButton).whileTrue(new Wrist_Setposition(m_wrist, Constants.A_Button));
        new Trigger(m_controller::getBButton).whileTrue(new Wrist_Setposition(m_wrist, Constants.B_Button));
        new Trigger(m_controller::getYButton).whileTrue(new Wrist_Setposition(m_wrist, Constants.Y_Button));


       new Trigger(m_controller::getBackButton).whileTrue(new Arm_Extend(m_extend, true));
       new Trigger(m_controller::getStartButton).whileTrue(new Arm_Extend(m_extend, false));

       new Trigger(m_controller::getRightBumper).whileTrue(new Right_Bumper_True());
       new Trigger(m_controller::getLeftBumper).whileTrue(new Left_Bumper_True());



    //  new Trigger(m_controller::getXButton).whileTrue(new ScoreDefault(m_arm, m_wrist, true, Constants.ARM_SCORE_MIDDLE, Constants.WRIST_SCORE));
      // new Trigger(m_controller::getXButton).whileTrue(new ScoreDefault(m_arm, m_wrist, true, Constants.ARM_PICKUP_CONE, Constants.WRIST_PICKUP_CONE));
      // new Trigger(m_controller::getXButton).whileTrue(new ScoreDefault(m_arm, m_wrist, true, Constants.ARM_PICKUP_CONE_DOWN, Constants.WRIST_PICKUP_CONE_DOWN));
      // new Trigger(m_controller::getXButton).whileTrue(new ScoreDefault(m_arm, m_wrist, true, Constants.ARM_PICKUP_CONE_DOWN, Constants.WRIST_PICKUP_CUBE));
/* 
       new Trigger(m_controller::getXButton).whileTrue(new ScoreDefault(m_arm, m_wrist, false, Constants.ARM_DEFAULT, Constants.WRIST_DEFAULT));
       new Trigger(m_controller::getXButton).whileTrue(new ScoreDefault(m_arm, m_wrist, false, Constants.ARM_SCORE_HIGH, Constants.WRIST_SCORE));
       new Trigger(m_controller::getXButton).whileTrue(new ScoreDefault(m_arm, m_wrist, false, Constants.ARM_SCORE_MIDDLE, Constants.WRIST_SCORE));
       new Trigger(m_controller::getXButton).whileTrue(new ScoreDefault(m_arm, m_wrist, false, Constants.ARM_PICKUP_CONE, Constants.WRIST_PICKUP_CONE));
       new Trigger(m_controller::getXButton).whileTrue(new ScoreDefault(m_arm, m_wrist, false, Constants.ARM_PICKUP_CONE_DOWN, Constants.WRIST_PICKUP_CONE_DOWN));
       new Trigger(m_controller::getXButton).whileTrue(new ScoreDefault(m_arm, m_wrist, false, Constants.ARM_PICKUP_CONE_DOWN, Constants.WRIST_PICKUP_CUBE));
*/
 

       new Trigger(()-> m_controller2.getRightTriggerAxis() > 0.80).whileTrue(new Drive_Boost(s_Swerve));
       new Trigger(()-> m_controller2.getRightTriggerAxis() > 0.80).whileFalse(new Drive_Boost_Off(s_Swerve));
       new Trigger(()-> m_controller2.getLeftTriggerAxis() > 0.80).whileTrue(new Drive_AntiBoost(s_Swerve));
       new Trigger(()-> m_controller2.getLeftTriggerAxis() > 0.80).whileFalse(new Drive_AntiBoost_Off(s_Swerve));

       new Trigger(m_controller2::getAButton).whileTrue(new CANdle_Orange_Command(m_candle));
       new Trigger(m_controller2::getBButton).whileTrue(new CANdle_Purple_Command(m_candle));


        // Configure the button bindings
        configureButtonBindings();
    }


    private double modifyAxis(double value) 
    {
        return value;
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() 
    {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() 
    {
        // An ExampleCommand will run in autonomous
        switch (a_chooser.getSelected()) 
        {
        case 1: return new exampleAuto(s_Swerve);
        case 2: return new Balance_Auto(m_lift, s_Swerve, m_auton, m_arm, m_intake);
        case 3: return new Blue_Two_Piece_Auto(m_lift, s_Swerve, m_auton, m_arm, m_intake, m_extend, m_wrist);
        case 4: return new Red_Two_Piece_Auto(m_lift, s_Swerve, m_auton, m_arm, m_intake, m_extend, m_wrist);
        case 5: return new AutoScoreCone(m_arm, m_intake, m_extend, m_wrist);
        case 6: return new Nothing_Auto(s_Swerve);
        case 7: return new Emergency_Auto(m_arm, m_intake, m_extend, m_wrist, s_Swerve);

        default: return new Red_Two_Piece_Auto(m_lift, s_Swerve, m_auton, m_arm, m_intake, m_extend, m_wrist);
        }
    }
}

