package com.stuypulse.robot.util.logged;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.LogTable;

/**
 *
 *
 * <h2>Logged Signal</h2>
 *
 * <p>Wraps a single dynamically-added CTRE {@code StatusSignal<T>} together with its most recently
 * refreshed value, so it can be logged and replayed by AdvantageKit.
 *
 * @see LoggedTalonFX#addSignal(StatusSignal)
 * @see TalonFXInputs
 * @author Faizaan (https://github.com/Faizaan-J)
 */
public final class LoggedSignal<T> {
  private final StatusSignal<T> signal;
  private final String key;
  private T value;

  // add as field so autocloseable doesn't complain about resource leak
  private final Alert unsupportedTypeAlert;

  public LoggedSignal(StatusSignal<T> signal) {
    this.signal = signal;
    this.key = signal.getName();
    this.value = signal.getValue();

    if (!getIsSupportedType(value)) {
      this.unsupportedTypeAlert =
          new Alert(
              "LoggedSignal",
              "Signal '"
                  + key
                  + "' has an unsupported type ("
                  + value.getClass().getName()
                  + "), so it will not be replayable.",
              AlertType.kWarning);
      unsupportedTypeAlert.set(true);
    } else {
      this.unsupportedTypeAlert = null;
    }
  }

  private static boolean getIsSupportedType(Object value) {
    return value instanceof StructSerializable
        || value instanceof Boolean
        || value instanceof Double
        || value instanceof Integer
        || value instanceof Long
        || value instanceof String
        || value instanceof Enum<?>;
  }

  /**
   * Returns the CTRE status signal.
   *
   * @return the CTRE status signal
   */
  public StatusSignal<T> getSignal() {
    return signal;
  }

  /**
   * Returns the log table key for this signal.
   *
   * @return the log table key for this signal
   */
  public String getKey() {
    return key;
  }

  /**
   * Returns the most recently refreshed value of the signal.
   *
   * @return the most recently refreshed value of the signal
   */
  public T getValue() {
    return value;
  }

  /** Pulls the latest value out of the (already-refreshed) underlying signal. */
  public void refresh() {
    value = signal.getValue();
  }

  /** Writes the current value into the given log table. */
  public void toLog(LogTable table) {
    Object _value = value;
    if (_value instanceof StructSerializable struct) {
      table.put(key, struct);
    } else if (_value instanceof Boolean bool) {
      table.put(key, bool.booleanValue());
    } else if (_value instanceof Double doublee) {
      table.put(key, doublee.doubleValue());
    } else if (_value instanceof Integer integer) {
      table.put(key, integer.intValue());
    } else if (_value instanceof Long longg) {
      table.put(key, longg.longValue());
    } else if (_value instanceof String string) {
      table.put(key, string);
    } else if (_value instanceof Enum<?> e) {
      table.put(key, e.name());
    } else {
      table.put(key, String.valueOf(_value));
      // final resort, but won't be reconstructable
      // in practice this should never happen unless ctre adds some new typa signals
    }
  }

  /** Reads (and overwrites) the current value from the given log table. */
  @SuppressWarnings({"unchecked", "rawtypes"})
  public void fromLog(LogTable table) {
    Object v = value;
    if (v instanceof StructSerializable s) {
      value = (T) table.get(key, s);
    } else if (v instanceof Boolean b) {
      value = (T) (Boolean) table.get(key, b.booleanValue());
    } else if (v instanceof Double d) {
      value = (T) (Double) table.get(key, d.doubleValue());
    } else if (v instanceof Integer i) {
      value = (T) (Integer) table.get(key, i.intValue());
    } else if (v instanceof Long l) {
      value = (T) (Long) table.get(key, l.longValue());
    } else if (v instanceof String s) {
      value = (T) table.get(key, s);
    } else if (v instanceof Enum<?> e) {
      String name = table.get(key, e.name());
      value = (T) Enum.valueOf((Class<Enum>) e.getDeclaringClass(), name);
    }
    // else then unsupported so it cant be replayed
  }
}
