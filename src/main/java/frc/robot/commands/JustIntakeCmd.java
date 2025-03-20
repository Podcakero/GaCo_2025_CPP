//
//  This command will set a triggering Event, and will end when that event has been processed.
//  Thus you can create a SerialCommandGroup full of triggeringEvents and they will execute one at a time.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TowerEvent;
import frc.robot.subsystems.TowerState;
import frc.robot.subsystems.TowerSubsystem;

public class JustIntakeCmd extends Command {
  TowerSubsystem tower;
  TowerEvent    event;

  /** Creates a new TriggerEvent. */
  public JustIntakeCmd(TowerSubsystem tower, TowerEvent event) {
    this.tower = tower;
    this.event = event;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tower.triggerEvent(event);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((tower.getState() == TowerState.INTAKE_PAUSE) || 
            (tower.getState() == TowerState.GOING_TO_SAFE)  ||  
            (tower.getState() == TowerState.GOT_CORAL));
  }
}
