package org.codeorange.utility.drivers;

/**
 * INSANELY CURSED, DO NOT USE
 *
 * @param <T> the inputs class attached to this subsystem
 */
public interface SubsystemIO<T extends SubsystemIO.SubsystemInputs> {
    interface SubsystemInputs {}

    default void updateInputs(T inputs) {}
}
