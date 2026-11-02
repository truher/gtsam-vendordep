package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;
import java.util.function.Function;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

/**
 * A pointer to a shared pointer.
 * 
 * Extends ForeignObject so that the shared pointer will be "deleted"
 * (decrementing its counter) when this container object is no longer reachable.
 * 
 * The underlying pointer must not be deleted (since it is shared), and so
 * generally should not extend ForeignObject.
 */
public class shared_ptr<T> extends ForeignObject {
    public enum FF {
        shared_ptr_get(ADDRESS, ADDRESS),
        shared_ptr_delete(null, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    final Function<MemorySegment, T> construct;

    /**
     * @param p    pointer to the shared_ptr itself.
     * @param ctor constructor of T, using the inner pointer (from get()).
     */
    public shared_ptr(//
            MemorySegment p, //
            Function<MemorySegment, T> ctor) {
        super(p, FF.shared_ptr_delete.h);
        construct = ctor;
    }

    /**
     * Instantiate T (which contains a pointer to the actual T) using
     * shared_ptr.get().
     * 
     * TODO: the pointer here must not be owned.
     * 
     * NULLABLE
     */
    public T get() {
        MemorySegment p = (MemorySegment) FF.shared_ptr_get.invoke(ptr);
        // transmute C++ nullptr to Java null.
        if (p.equals(MemorySegment.NULL))
            return null;
        return construct.apply(p);
    }

}
