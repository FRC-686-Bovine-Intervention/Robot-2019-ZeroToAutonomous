/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  private TalonSRX leftMotor1 = new TalonSRX(1); 
  private TalonSRX leftMotor2 = new TalonSRX(2); 
  private TalonSRX rightMotor1 = new TalonSRX(3); 
  private TalonSRX rightMotor2 = new TalonSRX(4); 

  private Joystick joy1 = new Joystick(0);

  private double startTime;

  @Override 
  public void robotInit() {
  }

  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
    leftMotor2.setNeutralMode(NeutralMode.Coast);
    rightMotor2.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void autonomousPeriodic() {
    double time = Timer.getFPGATimestamp();

if (time - startTime < 1){    
    leftMotor1.set(ControlMode.PercentOutput, 0.3);
    //leftMotor2.set(ControlMode.PercentOutput, 0.3);
    rightMotor1.set(ControlMode.PercentOutput, -0.3);
    //rightMotor2.set(ControlMode.PercentOutput, -0.3);
  } else{
    leftMotor1.set(ControlMode.PercentOutput, 0);
    //leftMotor2.set(ControlMode.PercentOutput, 0);
    rightMotor1.set(ControlMode.PercentOutput, 0);
    //rightMotor2.set(ControlMode.PercentOutput, 0);
  }
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    double speed = -joy1.getRawAxis(1) * 0.6;
    double turn = joy1.getRawAxis(0) * 0.3;

    double left = speed + turn;
    double right = speed - turn;
    
    leftMotor1.set(ControlMode.PercentOutput, left);
    //leftMotor2.set(ControlMode.PercentOutput, left);
    rightMotor1.set(ControlMode.PercentOutput, -right);
    //rightMotor2.set(ControlMode.PercentOutput, -right);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
