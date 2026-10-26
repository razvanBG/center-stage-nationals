package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

/**
 * Base class for every subsystem.
 *
 * The OpMode only writes the state it wants. {@link #update()} is called once per loop and
 * sends hardware commands only when the state has changed since the last call, which keeps
 * the loop fast. States that wait on a timer are listed in {@link #isTimed} and are
 * re-evaluated every loop until they move on by themselves.
 */
public abstract class StateController<S extends Enum<S>> {
    public S state;
    private S appliedState = null;

    protected StateController(S initialState) {
        state = initialState;
    }

    public void update() {
        everyLoop();
        if (state != appliedState || isTimed(state)) {
            appliedState = state;
            apply(state);
        }
    }

    /** Work that has to run every loop, before the state is checked. */
    protected void everyLoop() {
    }

    protected boolean isTimed(S s) {
        return false;
    }

    protected abstract void apply(S s);
}
