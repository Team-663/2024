// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.lang.constant.Constable;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class intakeNoteXboxCmd extends Command {
  /** Creates a new intakeOneNote. */
  Shooter m_shooter;
  public intakeNoteXboxCmd(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_shooter.setArmPosition(ArmConstants.ARM_DOWN_ENCODER_VALUE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (m_shooter.IsShooterSpinningTooFastForIntake())
    {
      m_shooter.intakeNote(ArmConstants.INTAKE_MOTOR_SPEED_SLOWER);
    }
    else
    {
      m_shooter.intakeNote();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.CheckIfNoteInIntake();
  }
}
