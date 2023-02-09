// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;




/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  private final CANSparkMax m_leftMotor1 = new CANSparkMax(1, MotorType.kBrushed);
  private final CANSparkMax m_rightMotor1 = new CANSparkMax(2, MotorType.kBrushed);
  private final CANSparkMax m_leftMotor2 = new CANSparkMax(3, MotorType.kBrushed);
  private final CANSparkMax m_rightMotor2 = new CANSparkMax(4, MotorType.kBrushed);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor1, m_rightMotor1);
  private final Joystick m_stick = new Joystick(0);

  public final static CommandXboxController primaryDriver = new CommandXboxController(0);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor1.setInverted(true);

    m_rightMotor2.follow(m_rightMotor1);
    m_leftMotor2.follow(m_leftMotor1);

  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    m_robotDrive.arcadeDrive(primaryDriver.getLeftY(), primaryDriver.getRightX());
  }
}
