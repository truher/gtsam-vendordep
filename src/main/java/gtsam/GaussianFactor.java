package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.MemoryLayout;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.SegmentAllocator;
import java.lang.foreign.StructLayout;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;

import org.team100.foreign.Lib;

public class GaussianFactor {
    private static final StructLayout PtrPair = MemoryLayout.structLayout(
            ValueLayout.ADDRESS.withName("first"),
            ValueLayout.ADDRESS.withName("second"));
    private static final VarHandle first = PtrPair.varHandle(MemoryLayout.PathElement.groupElement("first"));
    private static final VarHandle second = PtrPair.varHandle(MemoryLayout.PathElement.groupElement("second"));
    private static final MethodHandle GaussianFactor_jacobian = Lib.linker.downcallHandle(
            Lib.lib.findOrThrow("GaussianFactor_jacobian"),
            FunctionDescriptor.of(PtrPair, ADDRESS));
    /**
     * Pointer to a shared pointer that points at the factor, because that's how
     * these are produced, e.g. by NonlinearFactor.linearize().
     */
    final MemorySegment sharedPtrPtr;

    GaussianFactor(MemorySegment p) {
        sharedPtrPtr = p;
    }

    public Pair<Matrix, Vector> jacobian() throws Throwable {
        MemorySegment resultStruct = (MemorySegment) GaussianFactor_jacobian.invokeExact(
                (SegmentAllocator) Lib.arena, sharedPtrPtr);
        MemorySegment firstPtr = (MemorySegment) first.get(resultStruct, 0);
        MemorySegment secondPtr = (MemorySegment) second.get(resultStruct, 0);
        return new Pair<>(new Matrix(firstPtr), new Vector(secondPtr));
    }

}
