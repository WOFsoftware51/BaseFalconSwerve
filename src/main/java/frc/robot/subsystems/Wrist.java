// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Global_Variables;

public class Wrist extends SubsystemBase 
{  
  private final TalonFX _wrist = new TalonFX(Constants.Intake_.Wrist_Motor);
  private CANCoder wristCANCoder = new CANCoder(Constants.Wrist_CANCoder);

  
  private static TalonFX[] _instruments = {};

  public TalonFX[] returnArmMotors()
  {
    _instruments[0] = _wrist;
    return _instruments;
  }
  
  /** Creates a new wrist. */
  public void wrist_init() 
  {
    _wrist.setNeutralMode(NeutralMode.Brake);
    _wrist.setInverted(false);
    _wrist.setSensorPhase(true);
    wristCANCoder.configSensorDirection(true);
    wristCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    if(wrist_CANCoder()!=0)
    {
      updateEncoder();
    }
    else
    {
      _wrist.setSelectedSensorPosition(0);  //  angle*2048*100(gear ratio)/360
    }

        _wrist.config_kF(0, 0.025 , 30);
    _wrist.config_kP(0, 0.015, 30);   //.0.15
    _wrist.config_kI(0, 0.00015, 30);
    _wrist.config_kD(0, 0.5, 30);
   
    _wrist.configMotionCruiseVelocity(20000);
    _wrist.configMotionAcceleration(20000);
    _wrist.configMotionSCurveStrength(4);
      }

  
  /**Wrist On */
   public void Wrist_On(Double speed)
  {
    _wrist.set(ControlMode.PercentOutput, Global_Variables.robot_direction*speed);
  }

  public void Wrist_Off()
  {
    _wrist.set(ControlMode.PercentOutput,0.0);  
  }

  public void Wrist_Goto_Angle(double angle)
  {
  
      _wrist.set(ControlMode.MotionMagic, angle * Constants.WRIST_CONVERSION);    
  }

  public double wrist_encoder()
  {
    double wrist_coder = _wrist.getSelectedSensorPosition()/Constants.WRIST_CONVERSION;
    return wrist_coder;
  }

  public double wrist_CANCoder()
  {
    double wrist_CANCoder = wristCANCoder.getAbsolutePosition();
    return wrist_CANCoder;
  }
    public double wrist_Speed() 
    {
      return (_wrist.getSelectedSensorVelocity());
    }

public void updateEncoder()
{
  if(wrist_CANCoder()!=0)
  {
    if(wristCANCoder.getAbsolutePosition() < 0)
    {
      _wrist.setSelectedSensorPosition(((Constants.WRIST_OFFSET-(wristCANCoder.getAbsolutePosition()))-360)*Constants.WRIST_CONVERSION);  //  angle*2048*100(gear ratio)/360
    }
    else
    {
    _wrist.setSelectedSensorPosition((Constants.WRIST_OFFSET-(wristCANCoder.getAbsolutePosition()))*Constants.WRIST_CONVERSION);  //  angle*2048*100(gear ratio)/360
    }
  }
}

public int getAllianceColor()
{
  if(DriverStation.getAlliance()==Alliance.Blue)
  {
    return -1;
  }
  else if(DriverStation.getAlliance()==Alliance.Red)
  {
    return 1;
  }
  else
  {
    return 0;
  } 
}
   

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}
