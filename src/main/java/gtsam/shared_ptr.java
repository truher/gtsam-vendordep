package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;
import java.util.function.Function;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

/**
 * A pointer to a shared pointer. The shared pointer can be "deleted"
 * (decrementing its counter), but the underlying pointer must not be
 * deleted (since it is shared).
 * 
 * ForeignObject handles the deleting of the shared pointer.
 */
public class shared_ptr<T> extends ForeignObject {
    public enum FF {
        shared_ptr_get(ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    final MemorySegment sharedPtrPtr;
    final Function<MemorySegment, T> construct;

    /**
     * @param ptrPtr pointer to the shared_ptr itself
     * @param ctor   constructor of T, using a pointer
     */
    shared_ptr(//
            MemorySegment ptrPtr, //
            Function<MemorySegment, T> ctor,
            MethodHandle deleter) {
        super(ptrPtr, deleter);
        sharedPtrPtr = ptrPtr;
        construct = ctor;
    }

    /**
     * Instantiate T (which contains a pointer to the actual T) using
     * shared_ptr.get().
     * 
     * TODO: the pointer here must not be owned.
     */
    public T get() throws Throwable {
        return construct.apply((MemorySegment) FF.shared_ptr_get.h.invokeExact(sharedPtrPtr));
    }
}
