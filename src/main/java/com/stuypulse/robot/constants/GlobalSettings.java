package com.stuypulse.robot.constants;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;

public interface GlobalSettings {
  final CANBus RIO = new CANBus("rio");
  final CANBus CANIVORE = new CANBus("CANIVORE");

  // Add subsystems as they're added.
  public interface EnabledSubsystems {
    final LoggedNetworkBoolean VISION =
        new LoggedNetworkBoolean("/Tuning/Enabled Subsystems/Vision", true);
  }

  final Time DT = Milliseconds.of(20);

  final Mode simMode = Mode.SIM;
  final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
