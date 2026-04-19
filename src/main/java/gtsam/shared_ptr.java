package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;
import java.util.function.Function;

import org.team100.foreign.Lib;

public class shared_ptr<T> {
    private static final MethodHandle shared_ptr_get = Lib.linker.downcallHandle(
            Lib.lib.findOrThrow("shared_ptr_get"),
            FunctionDescriptor.of(ADDRESS, ADDRESS));
    final MemorySegment sharedPtrPtr;
    final Function<MemorySegment, T> f;

    /**
     * @param p pointer to the shared_ptr itself
     * @param f constructor of T, using a pointer
     */
    shared_ptr(MemorySegment p, Function<MemorySegment, T> f) {
        sharedPtrPtr = p;
        this.f = f;
    }

    /**
     * Instantiate T (which contains a pointer to the actual T) using
     * shared_ptr.get().
     * 
     * TODO: the pointer here must not be owned.
     */
    public T get() throws Throwable {
        return f.apply((MemorySegment) shared_ptr_get.invokeExact(sharedPtrPtr));
    }
}
