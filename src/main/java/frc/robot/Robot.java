// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot;



import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  private final WPI_TalonSRX m_leftMotor1 = new WPI_TalonSRX(1);
  private final WPI_TalonSRX m_rightMotor1 = new WPI_TalonSRX(2);
  private final WPI_TalonSRX m_leftMotor2 = new WPI_TalonSRX(3);
  private final WPI_TalonSRX m_rightMotor2 = new WPI_TalonSRX(4);

  private final DifferentialDrive robotDrive = new DifferentialDrive(m_leftMotor1, m_rightMotor1);
  
  public final static CommandXboxController primaryDriver = new CommandXboxController(0);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to inve`rt the left side instead.
    m_rightMotor1.setInverted(true);

    m_rightMotor2.follow(m_rightMotor1);
    m_leftMotor2.follow(m_leftMotor1);

    

  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    robotDrive.arcadeDrive(primaryDriver.getLeftY(), primaryDriver.getRightX());
  }
}