package org.team100.foreign;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;
import java.lang.ref.Cleaner;

/**
 * This has a pointer to a heap-allocated object in C++.
 * Deletes it when this gets garbage-collected.
 */
public abstract class ForeignObject {
    private record Cleanup(MethodHandle deleter, MemorySegment pointer) implements Runnable {
        public Cleanup {
            if (deleter == null || pointer == null)
                throw new IllegalArgumentException();
        }

        @Override
        public void run() {
            try {
                // System.out.println("Cleanup running...");
                deleter.invokeExact(pointer);
            } catch (Throwable e) {
            }
        }
    }

    public static final Cleaner CLEANER = Cleaner.create();

    /** The address of the referent. */
    public final MemorySegment ptr;

    /**
     * @param pointer Points to the referent, usually returned by "new".
     * @param deleter Calls "delete" on the pointer. Pass null for "observer"
     *                cases, so the pointer is not deleted.
     */
    protected ForeignObject(MemorySegment pointer, MethodHandle deleter) {
        ptr = pointer;
        if (deleter != null)
            CLEANER.register(this, new Cleanup(deleter, ptr));
    }
}
