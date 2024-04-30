// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private CANSparkMax m_rotationMotor;
  private DutyCycleEncoder m_encoder;

  private static final double M_TARGET_ANGLE = 90, KP = 0, KI = 0, KD = 0;
  private static final int ROTATION_MOTOR_ID = 0, ENCODER_ID = 0;
  private static double prevAngle = 0, sum = 0;



  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_rotationMotor = new CANSparkMax(ROTATION_MOTOR_ID, MotorType.kBrushless);
    m_encoder = new DutyCycleEncoder(ROTATION_MOTOR_ID);
    m_encoder.setDistancePerRotation(360);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    double currentAngle = (m_encoder.getDistance() % 360);

    double error = M_TARGET_ANGLE - currentAngle;
    double p = error * KP;

    sum += currentAngle;
    double i = sum * KI;

    double change = currentAngle - prevAngle;
    double d = change * KD;

    double pid = p + i + d;
    prevAngle = currentAngle;

    m_rotationMotor.set(pid);
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
