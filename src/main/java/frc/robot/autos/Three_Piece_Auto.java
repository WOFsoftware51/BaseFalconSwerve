package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.commands.Auton_Arm_Extend;
import frc.robot.commands.Auton_Intake;
import frc.robot.commands.Auton_Intake_Piece;
import frc.robot.commands.Auton_Wait;
import frc.robot.commands.ScoreMiddle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extend;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;


import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Three_Piece_Auto extends SequentialCommandGroup 
{
    private final Arm m_arm;
    private final Wrist m_wrist;
    private final Intake m_intake;
    private final Extend m_extend;
    private final Swerve s_Swerve;
    
    public Three_Piece_Auto(Swerve swerve, Extend extend, Arm arm, Wrist wrist, Intake intake)
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
   
        PathPlannerTrajectory Path = PathPlanner.loadPath("Cube_Pickup", new PathConstraints(4, 3));
        PathPlannerTrajectory Path2 = PathPlanner.loadPath("Cone_Pickup", new PathConstraints(4, 3));
        PathPlannerTrajectory Path3 = PathPlanner.loadPath("Cube_Score2", new PathConstraints(4, 3));
        HashMap<String, Command> eventMap = new HashMap<>();
        HashMap<String, Command> eventMap2 = new HashMap<>();
 
        FollowPathWithEvents command1 = new FollowPathWithEvents(
            s_Swerve.followTrajectoryCommand(Path, true), 
            Path.getMarkers(), 
            eventMap
            );

        FollowPathWithEvents command2 = new FollowPathWithEvents(
            s_Swerve.followTrajectoryCommand(Path2, true), 
            Path.getMarkers(), 
            eventMap2
            );


        FollowPathWithEvents command3 = new FollowPathWithEvents(
            s_Swerve.followTrajectoryCommand(Path3, true), 
            Path.getMarkers(), 
            eventMap2
            );    

        addCommands(
            new ParallelRaceGroup(
                new Auton_Arm_Extend(m_extend, Constants.EXTEND_SCORE_HIGH), 
                new ScoreMiddle(m_arm, Constants.ARM_SCORE_HIGH, m_wrist, Constants.WRIST_SCORE),
                new Auton_Wait(100)),
            new Auton_Intake(m_intake, 20, false),
            new ParallelRaceGroup(
                new Auton_Arm_Extend(m_extend, 0), 
                new ScoreMiddle(m_arm, 0, m_wrist, 0), 
                new Auton_Wait(60)),
            new ParallelCommandGroup(
            command1,
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new Auton_Arm_Extend(m_extend,  0), 
                    new ScoreMiddle(m_arm, Constants.ARM_PICKUP_CUBE, m_wrist, Constants.WRIST_PICKUP_CUBE), 
                    new Auton_Intake_Piece(m_intake, 150, true)),
                new ParallelRaceGroup(
                    new Auton_Arm_Extend(m_extend,  0), 
                    new ScoreMiddle(m_arm, 0, m_wrist, 0), 
                    new Auton_Intake(m_intake, 100, true))
                )
            ),
            new ParallelRaceGroup(
                new Auton_Arm_Extend(m_extend, Constants.EXTEND_SCORE_HIGH), 
                new ScoreMiddle(m_arm, Constants.ARM_SCORE_HIGH, m_wrist, Constants.WRIST_SCORE), 
                new Auton_Wait(120)), 
            new Auton_Intake(m_intake, 20, false),
            new ParallelRaceGroup(
                new Auton_Arm_Extend(m_extend, 0), 
                new ScoreMiddle(m_arm, 0, m_wrist, 0), 
                new Auton_Wait(60)),
            new ParallelCommandGroup(
            command2,
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new Auton_Arm_Extend(m_extend,  0), 
                    new ScoreMiddle(m_arm, Constants.ARM_PICKUP_CUBE, m_wrist, Constants.WRIST_PICKUP_CONE), 
                new Auton_Intake(m_intake, 150, true)),
                new ParallelRaceGroup(
                    new Auton_Arm_Extend(m_extend,  0), 
                    new ScoreMiddle(m_arm, 0, m_wrist, 0), 
                    new Auton_Intake(m_intake, 100, true))
                )
            ),
            command3,
            new ParallelRaceGroup(
                new Auton_Arm_Extend(m_extend, Constants.EXTEND_SCORE_HIGH), 
                new ScoreMiddle(m_arm, Constants.ARM_SCORE_HIGH, m_wrist, Constants.WRIST_SCORE), 
                new Auton_Wait(120)),
            new ParallelCommandGroup(
                new Auton_Arm_Extend(m_extend, Constants.EXTEND_SCORE_HIGH), 
                new ScoreMiddle(m_arm, Constants.ARM_SCORE_HIGH, m_wrist, Constants.WRIST_SCORE), 
                new Auton_Intake(m_intake, 20, false)
            )

        );
    }
}