package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
    //private final int robotCentric = XboxController.Button.kLeftBumper.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(m_controller2, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(m_controller2, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Intake m_intake = new Intake();
   // private final CANdle m_candle = new CANdle();
     private final Lift_Pneumatic m_lift = new Lift_Pneumatic();
     private final Arm m_arm = new Arm();
     private final Auton_Subsystem m_auton = new Auton_Subsystem();


    



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() 
    {

        SmartDashboard.putData("Auton", a_chooser);

        a_chooser.setDefaultOption("Test_Auto", 1);
        a_chooser.addOption("Balance", 2);
        a_chooser.addOption("Two Piece", 3);

        s_Swerve.setDefaultCommand(
            new TeleopSwerve
            (
                s_Swerve, 
                () -> m_controller2.getRawAxis(translationAxis), 
                () -> m_controller2.getRawAxis(strafeAxis), 
                () -> m_controller2.getRawAxis(rotationAxis),
                () -> robotCentric.getAsBoolean()
            )
        ); 
        
        m_arm.setDefaultCommand(new Arm_Command(m_arm, () -> -modifyAxis(m_controller.getLeftY())));

    //    m_candle.setDefaultCommand(new CANdle_Command(m_candle));
       new Trigger(m_controller::getAButton).whileTrue(new Intake_Command(m_intake));
       new Trigger(m_controller::getXButton).whileTrue(new Intake_Reverse_Command(m_intake));

       // new Trigger(m_controller::getXButton).whileFalse(new Intake_Command_Off(m_intake));
       // new Trigger(m_controller::getBButton).toggleOnTrue(new CANdle_Command(m_candle));
        new Trigger(m_controller::getBButton).whileTrue(new Lift_Pneumatic_On_Command(m_lift));
        new Trigger(m_controller::getYButton).whileTrue(new Lift_Pneumatic_Off_Command(m_lift));
        new Trigger(()-> m_controller2.getRightTriggerAxis() > 0.80).whileTrue(new Drive_Boost(s_Swerve));
        new Trigger(()-> m_controller2.getRightTriggerAxis() > 0.80).whileFalse(new Drive_Boost_Off(s_Swerve));
      // new Trigger(m_controller2::getAButton).toggleOnTrue((new Drive_Boost(s_Swerve)));   
        new Trigger(()-> m_controller2.getLeftTriggerAxis() > 0.80).whileTrue(new Drive_AntiBoost(s_Swerve));
        new Trigger(()-> m_controller2.getLeftTriggerAxis() > 0.80).whileFalse(new Drive_AntiBoost_Off(s_Swerve));


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
        case 1: return new Balance_Auto(m_lift, s_Swerve, m_auton, m_arm, m_intake);
        case 2: return new exampleAuto(s_Swerve);
        case 3: return new Two_Piece_Auto(m_lift, s_Swerve, m_auton, m_arm, m_intake);
        default: return new exampleAuto(s_Swerve);
        }
        
        // new Lift_Auto(m_lift, s_Swerve);
    }
}

