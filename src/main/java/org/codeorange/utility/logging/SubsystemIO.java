package org.codeorange.utility.logging;

public interface SubsystemIO<T> {
    default void updateInputs(T inputs) {}
}
