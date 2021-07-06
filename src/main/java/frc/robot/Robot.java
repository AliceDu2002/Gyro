// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
  // declare a chassis with two VivtorSPX motors that connected via PWM
  // VictorSPX can be connected via both PWM or CAN
  // You can either declare this way:
  /*
  private final PWMVictorSPX l_motor = new PWMVictorSPX(0);
  private final PWMVictorSPX r_motor = new PWMVictorSPX(1);
  private final DifferentialDrive m_drive = new DifferentialDrive(l_motor, r_motor);
  */
  private final Joystick m_stick = new Joystick(0);
  // declare a joystick, look up its port on driver station

  AHRS m_ahrs; // declare our gyro
  // don't forget to manage vendor libraries -> install libraries(offline)
  double kp = 0.003; // our tuning constant for going straight

  PIDController turnController; // declare a PID controller
  double KP = 0.002;
  double KI = 0.00;
  double KD = 0.00;
  // parameters for PID
  // tuning method: try
  double degreeTolerance = 2.0;
  // the error you can stand
  // when you approach your setpoint, your robot move slower or even can't move
  // or when it move too fast it might always overshoot
  // thus set the error you can tolerate to make robot easier to achieve your goal

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_ahrs = new AHRS(I2C.Port.kMXP);
    // initialize your gyro
    // it has three way to connect: I2C, SPI and Serial Port
    m_ahrs.enableLogging(true); // start the gyro
    m_ahrs.calibrate(); // calibrate: do once to set current speed and angle to zero
    m_ahrs.reset(); // set current angle to zero

    turnController = new PIDController(KP, KI, KD); // set the PID parameter
    turnController.enableContinuousInput(-180f, 180f); // -180 and 180 mean the same place -> continuous
    turnController.setIntegratorRange(-0.5, 0.5); // set how much you can "add" during every calculation
    turnController.setTolerance(degreeTolerance); // set tolerance (see the explanation above)
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_ahrs.reset(); 
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // code for going straight
    // use the idea of PID instead of the PID Controller class FIRST gives
    double error = -m_ahrs.getYaw();
    // our target angle is zero so use 0 to minus current angle and get the error
    m_robotDrive.tankDrive(0.4 + kp * error, 0.4 - kp * error);
    // fix the left and right motor speed according to the error
    // tankDrive: give left and right motor speed respectively
    // ArcadeDrive: give the speed of moving forward and turning 
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX()); // use Joystick to control robot
    SmartDashboard.putNumber("getYaw", m_ahrs.getYaw());
    // print gyro angle
    // other functions: getPitch(), getRoll()
    if(m_stick.getRawButton(1)) {
      m_robotDrive.arcadeDrive(0, MathUtil.clamp(turnController.calculate(0, m_ahrs.getYaw()), -1.0, 1.0));
    }
    // feed your setpoint and your current angle and it will calculate the value to be add on your motors
    // clamp: set the output range between -1 and 1
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
