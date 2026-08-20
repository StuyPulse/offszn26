package com.stuypulse.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public interface GlobalSettings {
  CANBus RIO = new CANBus("rio");
  CANBus CANIVORE = new CANBus("CANIVORE");

  // Add subsystems as they're added.
  public interface EnabledSubsystems {
    LoggedNetworkBoolean VISION =
        new LoggedNetworkBoolean("/Tuning/Enabled Subsystems/Vision", true);
    LoggedNetworkBoolean SHOOTER =
        new LoggedNetworkBoolean("/Tuning/Enabled Subsystems/Shooter", true);
  }

  Time DT = Milliseconds.of(20);

  Mode SIMULATION_TASK =
      Mode.SIM; // What to do during simulation mode. Change this to REPLAY when replaying. Change
  // to SIM when simulating code.
  Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIMULATION_TASK;
  VisionMode VISION_MODE = VisionMode.LIMELIGHT_VISION;

  enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  enum VisionMode {
    LIMELIGHT_VISION,
    PHOTON_VISION
  }
}
