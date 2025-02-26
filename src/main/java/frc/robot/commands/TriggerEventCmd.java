//
//  This command will set a triggering Event, and will end when that event has been processed.
//  Thus you can create a SerialCommandGroup full of triggeringEvents and they will execute one at a time.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TowerEvent;
import frc.robot.subsystems.TowerSubsystem;

public class TriggerEventCmd extends Command {
  TowerSubsystem tower;
  TowerEvent    event;

  /** Creates a new TriggerEvent. */
  public TriggerEventCmd(TowerSubsystem tower, TowerEvent event) {
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
    return (tower.getPendingEvent() == TowerEvent.NONE);
  }
}
