// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Extend extends SubsystemBase 
{
private final TalonFX _extend = new TalonFX(Constants.Extend_Motor);

  /** Creates a new extend. */
  public void extend_init()
  {
    _extend.setNeutralMode(NeutralMode.Brake);
    _extend.setInverted(false);
    _extend.setSensorPhase(true);
    _extend.config_kF(0, 0.07 , 30);
    _extend.config_kP(0, 0.03, 30);   //.0.15
    _extend.config_kI(0, 0.0, 30);
    _extend.config_kD(0, 0, 30);
    _extend.configMotionCruiseVelocity(20000);
    _extend.configMotionAcceleration(15000);
    _extend.configMotionSCurveStrength(4);
  }

 public void extend_resetEncoder()
 {
   _extend.setSelectedSensorPosition(0);
 }

 public void Extend_Goto_Length(double value)
 {
  _extend.set(ControlMode.MotionMagic, value);  //Don't know the position right now
 }

  public void extend_on()
  {
    _extend.set(ControlMode.PercentOutput, 0.2);  
  }

  public void extend_reverse()
  {
    _extend.set(ControlMode.PercentOutput, -0.2);  
  }

  public void extend_off()
  {
    _extend.set(ControlMode.PercentOutput, 0.0);  
  }

  public double extend_speed()
  {
    double extend_speed = _extend.getSelectedSensorVelocity();
    return extend_speed;
  }

  public double extend_encoder()
  {
    double extend_encoder = _extend.getSelectedSensorPosition();
    return extend_encoder;
  }

  

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Extend_Encoder", extend_encoder());
    SmartDashboard.putNumber("Forward Limit", _extend.isFwdLimitSwitchClosed());
    SmartDashboard.putNumber("Rev L imit", _extend.isRevLimitSwitchClosed());

    if(_extend.isFwdLimitSwitchClosed()==1)
    {
      extend_resetEncoder();
    }

  }
}
