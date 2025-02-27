package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    // private final SendableChooser<Boolean> songBool = new SendableChooser<>();


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
    private final Arm m_arm = new Arm();
    private final Auton_Subsystem m_auton = new Auton_Subsystem();
    private final Extend m_extend = new Extend();
    private final Wrist m_wrist = new Wrist();





    



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() 
    {

        SmartDashboard.putData("Auton", a_chooser);
    
        a_chooser.setDefaultOption("Score Cone Balance", 8);
        a_chooser.addOption("Test", 2);
        a_chooser.addOption("2 piece Auto Blue", 1);
        a_chooser.addOption("2 piece Auto Red", 15);
        a_chooser.addOption("3 piece Auto Blue", 16);
        a_chooser.addOption("2 piece Auto Blue Bump", 17);
        a_chooser.addOption("2 piece Auto Red Bump", 18);

        a_chooser.addOption("Sit", 6);
        a_chooser.addOption("Score Cone", 5);
        a_chooser.addOption("Score Cone Balance Right Cube", 9);
        a_chooser.addOption("Score Cone Balance Left Cube", 21);

    //    a_chooser.addOption("PathWeaver Auto", 10);
       // a_chooser.addOption("Two piece Auto", 11);
        //a_chooser.addOption("Leave Zone Auto", 12);
        a_chooser.addOption("Leave Zone No Cube ", 7);
        a_chooser.addOption("Score Cube ", 14);
        a_chooser.addOption("Testing ", 19);
        a_chooser.addOption("Score Cone + Leave Zone ", 20);
        a_chooser.addOption("Limelight Auto", 22);
        a_chooser.addOption("Limelight 3 Piece Auto Blue", 23);
        a_chooser.addOption("Score Balance Mobility Right", 24);

        // SmartDashboard.putData("Song Bool", songBool);
        // songBool.setDefaultOption("Off", false);
        // songBool.addOption("On", true);


        SmartDashboard.putData("Auton Arm", Global_Variables.a_chooser_Arm);
/* 
        for(Double i=0.0; i<10.0; i++)
        {
            Global_Variables.a_chooser_Arm.setDefaultOption(Double.toString(i), -i);
        }
 */

 
        SmartDashboard.putData("Songs", Global_Variables.song);

        Global_Variables.song.setDefaultOption("DMX", "DMX.chrp");
        Global_Variables.song.setDefaultOption("Canada", "canada.chrp");
        Global_Variables.song.setDefaultOption("Lofi Girl (Hopefully)", "Lofi_Girl.chrp");



        s_Swerve.setDefaultCommand
        (
            new TeleopSwerve
            (
                s_Swerve, 
                () -> m_controller2.getRawAxis(translationAxis), 
                () -> m_controller2.getRawAxis(strafeAxis), 
                () -> m_controller2.getRawAxis(rotationAxis),
                () -> false         //robotCentric.getAsBoolean()
            )
         );
     //  new Trigger(m_controller2::getAButton).whileTrue(new CANdle_Orange_Command(m_candle));  
     //  new Trigger(m_controller2::getBButton).whileTrue(new CANdle_Purple_Command(m_candle)); 

       m_candle.setDefaultCommand(new CANdle_Default(m_candle));
       m_arm.setDefaultCommand(new Arm_Command(m_arm, () -> modifyAxis(m_controller.getLeftY())));
       m_wrist.setDefaultCommand(new Wrist_Command (() -> modifyAxis(m_controller.getRightY()), m_wrist));


       m_wrist.setDefaultCommand(new Wrist_Command((() -> (m_controller.getRightY())), m_wrist));


       new Trigger(()-> m_controller.getLeftTriggerAxis() > 0.80).whileTrue(new Intake_Command(m_intake));
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

        new Trigger(m_controller::getYButton).whileTrue(new CANdle_Intake(m_candle, Constants.Y_Button));
        new Trigger(m_controller::getBButton).whileTrue(new CANdle_Intake(m_candle, Constants.B_Button));
        new Trigger(m_controller::getXButton).whileTrue(new CANdle_Intake(m_candle, Constants.X_Button));
        new Trigger(m_controller::getAButton).whileTrue(new CANdle_Intake(m_candle, Constants.A_Button));

        new Trigger(m_controller::getYButton).whileTrue(new  Intake_Command_YButton(m_intake));
        new Trigger(m_controller::getXButton).whileTrue(new  Intake_Command(m_intake));
        new Trigger(m_controller::getAButton).whileTrue(new Intake_Command_Cones(m_intake));
        new Trigger(m_controller::getBButton).whileTrue(new Intake_Command_Cones(m_intake));

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
 

     //  new Trigger(()-> m_controller2.getRightTriggerAxis() > 0.80).whileTrue(new Drive_Boost(s_Swerve));
       //new Trigger(()-> m_controller2.getRightTriggerAxis() > 0.80).whileFalse(new Drive_Boost_Off(s_Swerve));
       //new Trigger(()-> m_controller2.getLeftTriggerAxis() > 0.80).whileTrue(new Drive_AntiBoost(s_Swerve));
       //new Trigger(()-> m_controller2.getLeftTriggerAxis() > 0.80).whileFalse(new Drive_AntiBoost_Off(s_Swerve));

       new Trigger(()-> m_controller2.getRightTriggerAxis() > 0.80).whileTrue(new Right_Bumper_Boost_True());
       new Trigger(()-> m_controller2.getLeftTriggerAxis() > 0.80).whileTrue(new Left_Bumper_Boost_True());
       


       new Trigger(()-> m_controller2.getRightTriggerAxis() > 0.80).whileTrue(new CANdle_Rainbow_Command(m_candle));
       new Trigger(m_controller2::getRightBumper).whileTrue(new CANdle_Orange_Command(m_candle));
       new Trigger(m_controller2::getLeftBumper).whileTrue(new CANdle_Purple_Command(m_candle));
       // new Trigger(m_controller2::getXButton).whileTrue(new Music(s_Swerve, true));
       new Trigger(m_controller2::getAButton).whileTrue(new Music(s_Swerve, m_arm, m_intake, m_extend, m_wrist, true));




       new Trigger(m_controller2::getYButton).whileTrue(new RunCommand(() -> m_intake.Intake_Reverse_Fast()));
 //      new Trigger(m_controller2::getRightBumper).whileTrue(new CANdle_Intake(m_candle, Constants.A_Button));
 //      new Trigger(m_controller2::getLeftBumper).whileTrue(new CANdle_Intake(m_candle, Constants.X_Button));
        
      // new Trigger(m_controller2::getAButton).whileTrue(new Limelight_Change_Pipeline(s_Swerve));


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
        case 1: return new Example_Auto(s_Swerve, m_extend, m_arm, m_wrist, m_intake);
        case 2: return new Balance_Auto(m_arm, m_intake, m_extend, m_wrist, s_Swerve);
        case 3: return new Blue_Two_Piece_Auto(s_Swerve, m_auton, m_arm, m_intake, m_extend, m_wrist);
        case 4: return new Red_Two_Piece_Auto(s_Swerve, m_auton, m_arm, m_intake, m_extend, m_wrist);
        case 5: return new AutoScoreCone(m_arm, m_intake, m_extend, m_wrist);
        case 6: return new Nothing_Auto(s_Swerve);
        case 7: return new Emergency_Auto(m_arm, m_intake, m_extend, m_wrist, s_Swerve);
        case 8: return new ScoreCone_Balance(m_arm, m_intake, m_extend, m_wrist, s_Swerve);
        case 9: return new ScoreCone_BalancePlus3(m_arm, m_intake, m_extend, m_wrist, s_Swerve);
        case 10: return new PathWeaver_Auto(s_Swerve, m_arm, m_intake, m_extend, m_wrist);
        case 11: return new Two_piece_AutoBAD(m_arm, m_intake, m_extend, m_wrist, s_Swerve);
        case 12: return new Leave_Zone_Auto(m_arm, m_intake, m_extend, m_wrist, s_Swerve);
        case 13: return new Two_Piece_AutoPlus(s_Swerve, m_extend, m_arm, m_wrist, m_intake);
        case 14: return new AutoScoreCube(m_arm, m_intake, m_extend, m_wrist, s_Swerve);
        case 15: return new Example_Auto_Red(s_Swerve, m_extend, m_arm, m_wrist, m_intake);
        case 16: return new Example_Auto_3Piece_Blue(s_Swerve, m_extend, m_arm, m_wrist, m_intake);
        case 17: return new Example_Auto_Blue_Bump(s_Swerve, m_extend, m_arm, m_wrist, m_intake);
        case 18: return new Example_Auto_Red_Bump(s_Swerve, m_extend, m_arm, m_wrist, m_intake);
        case 19: return new Test_Auto(s_Swerve);
        case 20: return new AutoScoreCone_LeaveZone(m_arm, m_intake, m_extend, m_wrist, s_Swerve);
        case 21: return new ScoreCone_BalancePlus3_Left(m_arm, m_intake, m_extend, m_wrist, s_Swerve);
        case 22: return new Limelight_Auto(s_Swerve);
        case 23: return new Example_Auto_Limelight(s_Swerve, m_extend, m_arm, m_wrist, m_intake);
        case 24: return new ScoreCone_BalanceMobility(s_Swerve, m_extend, m_arm, m_wrist, m_intake);

        
        default: return new ScoreCone_Balance(m_arm, m_intake, m_extend, m_wrist, s_Swerve);
        }
    }
}

