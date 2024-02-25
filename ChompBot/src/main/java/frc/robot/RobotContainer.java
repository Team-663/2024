// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.auto.Autos;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
//import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
//import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
   // The robot's subsystems and commands are defined here...
   private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
         "swerve"));

   private final Shooter m_shooter = new Shooter();
   // CommandJoystick rotationController = new CommandJoystick(1);
   // Replace with CommandPS4Controller or CommandJoystick if needed
   //CommandJoystick driverController = new CommandJoystick(1);

   // CommandJoystick driverController = new
   // CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
   CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.USB_PORT_XBOX_DRIVER);
   CommandXboxController operatorXbox = new CommandXboxController(OperatorConstants.USB_PORT_XBOX_OPERATOR);
   //XboxController operatorXbox = new XboxController(OperatorConstants.USB_PORT_XBOX_OPERATOR);
   //CommandXboxController operatorXbox = new CommandXboxController(OperatorConstants.USB_PORT_XBOX_OPERATOR);

   /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
   public RobotContainer() {
      // Configure the trigger bindings
      configureBindings();
      // 185.0 front left
      // angle gear ratio default 12.8
      //
      AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
            // Applies deadbands and inverts controls because joysticks
            // are back-right positive while robot
            // controls are front-left positive
            () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                  OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                  OperatorConstants.LEFT_X_DEADBAND),
               () -> -driverXbox.getLeftTriggerAxis()+driverXbox.getRightTriggerAxis(),
               () -> 0,
            //() -> -driverXbox.getRightX(),
            //() -> -driverXbox.getRightY(),
            false);

            TeleopDrive closedFieldRel = new TeleopDrive(
               drivebase,
               () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
               () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
               //() -> -driverXbox.getRawAxis(2)+driverXbox.getRawAxis(3),
               () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.LEFT_X_DEADBAND),
               () -> true, false, false);
       
      drivebase.setDefaultCommand(closedFieldRel);

      m_shooter.setDefaultCommand(m_shooter.intakeIdleCommand());

      //m_shooter.setDefaultCommand(m_shooter.armByXboxCommand(operatorXbox.getLeftY()));
      //   m_shooter.shooterByXboxCommand(
      //      () -> operatorXbox.getLeftY(),
      //      () -> -getOperatorTriggerCombined()
      //      ));
   }

   /**
    * Use this method to define your trigger->command mappings. Triggers can be
    * created via the
    * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
    * an arbitrary
    * predicate, or via the named factories in {@link
    * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
    * {@link
    * CommandXboxController
    * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
    * PS4} controllers or
    * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
    * joysticks}.
    */
   private void configureBindings() {
      // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
      driverXbox.start().onTrue(new InstantCommand(drivebase::zeroGyro));
      driverXbox.x().whileTrue(m_shooter.intakeNoteCommand());
      driverXbox.a().whileTrue(m_shooter.shootNoteCommandOpenLoop(0.5));
      driverXbox.rightTrigger(0.05).whileTrue(m_shooter.shootNoteCommandOpenLoop2(() -> driverXbox.getRightTriggerAxis()));
      // DONT THINK WE NEED THIS driverXbox.rightTrigger(0.05).whileFalse(m_shooter.intakeIdleCommand());
      
      driverXbox.leftTrigger(0.05).whileTrue(m_shooter.intakeManualSpeed(() -> -driverXbox.getLeftTriggerAxis()));
      
      operatorXbox.leftTrigger(0.05).whileTrue(m_shooter.armByXboxCommand(() -> operatorXbox.getLeftTriggerAxis()));
      operatorXbox.rightTrigger(0.05).whileTrue(m_shooter.armByXboxCommand(() -> -operatorXbox.getRightTriggerAxis()));
      //operatorXbox.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.15).onTrue(
      //   m_shooter.armByXboxCommand(operatorXbox.getLeftY()));

      //new JoystickButton(operatorXbox, 1).onTrue(new StartEndCommand(m_shooter::intakeNoteCommand, null,m_shooter));
      /* try using CommandJoystick...
      new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
      new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
      
      new JoystickButton(driverXbox, XboxController.Button.kX.value).onTrue(m_shooter.intakeNoteCommand());
      new JoystickButton(driverXbox, XboxController.Button.kA.value).onTrue(m_shooter.shootNoteCommand(0.75));
      */
   }

   /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
   public Command getAutonomousCommand() {
      // An example command will be run in autonomous
      return Autos.exampleAuto(drivebase);
   }

   public void setMotorBrake(boolean brake) {
      drivebase.setMotorBrake(brake);
   }

   private double getOperatorTriggerCombined()
   {
      return 0.0;
      //return operatorXbox.getLeftTriggerAxis() + operatorXbox.getRightTriggerAxis();
   }
   private double getDriverTriggerCombined()
   {
      return driverXbox.getLeftTriggerAxis() + driverXbox.getRightTriggerAxis();
   }
}
