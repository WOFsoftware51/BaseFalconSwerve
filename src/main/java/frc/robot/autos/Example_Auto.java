package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.Auton_Arm_Extend;
import frc.robot.commands.Auton_Intake;
import frc.robot.commands.Auton_Wait;
import frc.robot.commands.ScoreMiddle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extend;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Example_Auto extends SequentialCommandGroup 
{
    private final Arm m_arm;
    private final Wrist m_wrist;
    private final Intake m_intake;
    private final Extend m_extend;
    private final Swerve s_Swerve;
    
    public Example_Auto(Swerve swerve, Extend extend, Arm arm, Wrist wrist, Intake intake)
    {
        this.m_arm = arm;
        addRequirements(arm);
        this.m_wrist = wrist;
        addRequirements(wrist);
        this.m_extend = extend;
        addRequirements(extend);
        this.m_intake = intake;
        addRequirements(intake);
        this.s_Swerve = swerve;
        addRequirements(swerve);
   
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("2Piece", new PathConstraints(4, 3));
        PathPlannerTrajectory examplePath2 = PathPlanner.loadPath("DriveAway", new PathConstraints(4, 3));
       // PathPlannerState exampleState = (PathPlannerState) examplePath.sample(1.2);
    
    
    // This is just an example event map. It would be better to have a constant, global event map
    // in your code that will be used by all path following commands.

        addCommands(
            new ParallelRaceGroup(
                new Auton_Arm_Extend(m_extend, Constants.EXTEND_SCORE_HIGH+Constants.EXTEND_OFFSET), 
                new ScoreMiddle(m_arm, Constants.ARM_SCORE_HIGH+Constants.AUTON_ARM_OFFSET, m_wrist, Constants.WRIST_SCORE), // Constants.ARM_SCORE_HIGH-3
                new Auton_Wait(120)),
            new Auton_Intake(m_intake, 20, false),
            new ParallelRaceGroup(
                new Auton_Arm_Extend(m_extend, 0), 
                new ScoreMiddle(m_arm, 0, m_wrist, 0), 
                new Auton_Wait(60)),
            new ParallelCommandGroup(
                s_Swerve.followTrajectoryCommand(examplePath, true),
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new Auton_Arm_Extend(m_extend,  0), 
                        new ScoreMiddle(m_arm, Constants.ARM_PICKUP_CUBE, m_wrist, Constants.WRIST_PICKUP_CUBE), 
                        new Auton_Intake(m_intake, 150, true)),
                    new ParallelRaceGroup(
                        new Auton_Arm_Extend(m_extend,  0), 
                        new ScoreMiddle(m_arm, 0, m_wrist, 0), 
                        new Auton_Intake(m_intake, 100, true))
                )
            ),
            new ParallelRaceGroup(
                new Auton_Arm_Extend(m_extend, Constants.EXTEND_SCORE_HIGH), 
                new ScoreMiddle(m_arm, Constants.ARM_SCORE_HIGH-5, m_wrist, 80), 
                new Auton_Wait(100)), 
            new Auton_Intake(m_intake, 20, false),
            new ParallelRaceGroup(
                new Auton_Arm_Extend(m_extend, 0), 
                new ScoreMiddle(m_arm, 0, m_wrist, 0), 
                new Auton_Wait(60)),
            new ParallelCommandGroup(
                s_Swerve.followTrajectoryCommand(examplePath2, false),
                new ParallelRaceGroup(
                    new Auton_Arm_Extend(m_extend,  0), 
                    new ScoreMiddle(m_arm, -90, m_wrist, 0),
                    new Auton_Wait(100))
                )

            //, swerveControllerCommand2
        );
    }
}