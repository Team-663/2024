// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.intake.intakeBackupNote;
import frc.robot.commands.intake.intakeNoteXboxCmd;
import frc.robot.commands.intake.intakeOneNote;
import frc.robot.commands.shooter.shootNote;
import frc.robot.commands.shooter.shootNoteXboxCmd;
import frc.robot.commands.swervedrive.auto.Autos;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.commands.climber.climbByXbox;
import frc.robot.commands.shooter.setArmPositionCmd;
//import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
//import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.io.File;
import java.util.function.BooleanSupplier;
import edu.wpi.first.cameraserver.CameraServer;
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
   private final Climber m_climber = new Climber();

   CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.USB_PORT_XBOX_DRIVER);
   CommandXboxController operatorXbox = new CommandXboxController(OperatorConstants.USB_PORT_XBOX_OPERATOR);
    private final SendableChooser<Command> autoChooser;
   /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
   public RobotContainer() {
      // Configure the trigger bindings
      NamedCommands.registerCommand("ArmLow", m_shooter.setArmPositionCmd(ArmConstants.ARM_CLOSE_SHOT_SETPOINT));
      NamedCommands.registerCommand("ShootNote", new shootNote(m_shooter, 5000.0, () -> {return true;}));
      NamedCommands.registerCommand("intakeNote", new intakeOneNote(m_shooter));
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Mode", autoChooser);
      autoChooser.addOption("singleSide1Note", new PathPlannerAuto("singleSide1Note"));
      autoChooser.addOption("singleSide2Note", new PathPlannerAuto("singleSide2Note"));
      autoChooser.addOption("singleSide3Note", new PathPlannerAuto("singleSide3Note"));
      
      autoChooser.addOption("doubleSide1Note", new PathPlannerAuto("doubleSide1Note"));
      autoChooser.addOption("doubleSide2Note", new PathPlannerAuto("doubleSide2Note"));
      autoChooser.addOption("doubleSide3Note", new PathPlannerAuto("doubleSide3Note"));

      autoChooser.addOption("mid2Note", new PathPlannerAuto("mid2Note"));
      autoChooser.addOption("mid3NoteSingleSide", new PathPlannerAuto("mid3NoteSingleSide"));
      autoChooser.addOption("mid3NoteDoubleSide", new PathPlannerAuto("mid3NoteDoubleSide"));
      

      
      // 185.0 front left
      // angle gear ratio default 12.8
      //
      SmartDashboard.putData(m_shooter);
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
               () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
               () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
               //() -> -driverXbox.getRawAxis(2)+driverXbox.getRawAxis(3),
               () -> MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.LEFT_X_DEADBAND),
               () -> true, false, false);
       
      drivebase.setDefaultCommand(closedFieldRel);

      m_shooter.setDefaultCommand(m_shooter.intakeIdleCommand());
      m_climber.setDefaultCommand(m_climber.climberIdleCommand());
      configureBindings();
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
   private void configureBindings()
   {
      
      SmartDashboard.putData("Test Auto", new PathPlannerAuto("TestPathAuto"));
      
      // Driver Xbox controls swerve and Climber
      driverXbox.start().onTrue(new InstantCommand(drivebase::zeroGyro));

      driverXbox.a().whileTrue(new climbByXbox(m_climber
                                                            , () -> driverXbox.a().getAsBoolean()
                                                            , () -> driverXbox.getLeftTriggerAxis()
                                                            , () -> driverXbox.getRightTriggerAxis()
                                                            , () -> driverXbox.leftBumper().getAsBoolean()
                                                            , () -> driverXbox.rightBumper().getAsBoolean()
                                                            ));
                        
      driverXbox.a().negate().and(driverXbox.leftTrigger(0.05)).and(driverXbox.rightTrigger(0.5)).whileTrue(new climbByXbox(m_climber
                                                            , () -> true
                                                            , () -> driverXbox.getLeftTriggerAxis()
                                                            , () -> driverXbox.getRightTriggerAxis()
                                                            , () -> false
                                                            , () -> false
                                                            ));
      


      operatorXbox.x().whileTrue(new intakeNoteXboxCmd(m_shooter, true)).onFalse(new intakeBackupNote(m_shooter));

      
      operatorXbox.b().whileTrue(new SequentialCommandGroup(
                  m_shooter.setArmPositionCmd(ArmConstants.ARM_CLOSE_SHOT_SETPOINT)
                  ,new shootNoteXboxCmd(m_shooter, 5000.0, () -> {
                                                                        return (operatorXbox.getRightTriggerAxis() > 0.0 ? true : false);
                                                                        })));

      operatorXbox.y().whileTrue(new SequentialCommandGroup(
                  m_shooter.setArmPositionCmd(ArmConstants.ARM_LONG_SHOT_SETPOINT)
                  ,new shootNoteXboxCmd(m_shooter, 5000.0, () -> {
                                                                        return (operatorXbox.getRightTriggerAxis() > 0.0 ? true : false);
                                                                        })));

      operatorXbox.a().whileTrue(new SequentialCommandGroup(
         m_shooter.setArmPositionCmd(ArmConstants.ARM_AMP_SHOT_SETPOINT)
         ,m_shooter.intakeManualSpeed(() -> -operatorXbox.getLeftTriggerAxis(), () -> {return 0.0;})
      ));

      
      
      operatorXbox.povDown().whileTrue(m_shooter.setArmPositionCmd(2380));
      operatorXbox.povUp().whileTrue(m_shooter.setArmPositionCmd(3300));
      operatorXbox.leftTrigger(0.05).whileTrue(m_shooter.intakeManualSpeed(() -> -operatorXbox.getLeftTriggerAxis(), () -> -operatorXbox.getLeftTriggerAxis()));
      operatorXbox.axisGreaterThan(
                                    XboxController.Axis.kLeftY.value, 0.1).or(
                                    operatorXbox.axisLessThan(XboxController.Axis.kLeftY.value, -0.1)).whileTrue(
                                       m_shooter.armByXboxCommand(() -> -operatorXbox.getLeftY()));

      operatorXbox.leftBumper().whileTrue(new intakeNoteXboxCmd(m_shooter, false));

   }

   /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
   public Command getAutonomousCommand() {
      // An example command will be run in autonomous
      return autoChooser.getSelected();
      //return Autos.exampleAuto(drivebase);
   }

   public void setArmToHere()
   {
      m_shooter.setArmAtCurrentPosition();
   }

   public void setMotorBrake(boolean brake) {
      drivebase.setMotorBrake(brake);
   }
}
