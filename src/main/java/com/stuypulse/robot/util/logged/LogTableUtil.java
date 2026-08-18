package com.stuypulse.robot.util.logged;

import edu.wpi.first.util.struct.StructSerializable;
import org.littletonrobotics.junction.LogTable;

/**
 *
 *
 * <h2>Log Table Utility</h2>
 *
 * <p>Utility methods for reading and writing dynamically typed values to and from a {@code
 * LogTable}.
 *
 * <p>This centralizes type handling without having to duplicate the same if else logic in multiple
 * places.
 *
 * @author Faizaan (https://github.com/Faizaan-J)
 */
public final class LogTableUtil {
  private LogTableUtil() {}

  /**
   * Puts a value into a {@code LogTable} with the given key, automatically handling the type of the
   * value.
   *
   * @param table the {@code LogTable} to put the value into
   * @param key The key where the value will be stored.
   * @param value The value to store in the {@code LogTable}. If it's not a supported type, it will
   *     be converted to a string.
   */
  public static void put(LogTable table, String key, Object value) {
    if (value instanceof Boolean) {
      table.put(key, (Boolean) value);
    } else if (value instanceof Double) {
      table.put(key, (Double) value);
    } else if (value instanceof Integer) {
      table.put(key, (Integer) value);
    } else if (value instanceof Long) {
      table.put(key, (Long) value);
    } else if (value instanceof String) {
      table.put(key, (String) value);
    } else if (value instanceof Enum<?>) {
      table.put(key, ((Enum<?>) value).name());
    } else {
      // turn into string for unsupported types, can't be replayed tho.
      table.put(key, value.toString());
    }
  }

  /**
   * Gets a value from a {@code LogTable} with the given key, automatically handling the type of the
   * value based on the provided default value.
   *
   * @param <T> The type of the value.
   * @param table The {@code LogTable} to get the value from.
   * @param key The key where the valid is stored.
   * @param defaultValue The default value to return if the key does not exist in the {@code
   *     LogTable}.
   * @return The value from the {@code LogTable} if it exists, otherwise the default value.
   */
  @SuppressWarnings({"unchecked", "rawtypes"})
  public static <T> T get(LogTable table, String key, T defaultValue) {
    if (defaultValue instanceof StructSerializable struct) {
      return ((T) table.get(key, struct));
    } else if (defaultValue instanceof Boolean bool) {
      return ((T) Boolean.valueOf(table.get(key, bool)));
    } else if (defaultValue instanceof Double dbl) {
      return ((T) Double.valueOf(table.get(key, dbl)));
    } else if (defaultValue instanceof Integer integer) {
      return ((T) Integer.valueOf(table.get(key, integer)));
    } else if (defaultValue instanceof Long lng) {
      return ((T) Long.valueOf(table.get(key, lng)));
    } else if (defaultValue instanceof String string) {
      return ((T) table.get(key, string));
    } else if (defaultValue instanceof Enum<?> enumerator) {
      String name = table.get(key, enumerator.name());

      return ((T) Enum.valueOf((Class<Enum>) enumerator.getDeclaringClass(), name));
    }

    return defaultValue;
  }

  /**
   * Returns true if the value is a supported type for logging, false otherwise.
   *
   * @param value The value to check for logging support.
   * @return true if the value is a supported type for logging, false otherwise.
   */
  public static boolean isSupportedType(Object value) {
    return value instanceof StructSerializable
        || value instanceof Boolean
        || value instanceof Double
        || value instanceof Integer
        || value instanceof Long
        || value instanceof String
        || value instanceof Enum<?>;
  }
}
