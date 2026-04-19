package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class CustomFactor extends NonlinearFactor {
    private static final MethodHandle CustomFactor = Lib.down(
            "CustomFactor", ADDRESS, ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle CustomFactor_keys = Lib.down(
            "CustomFactor_keys", ADDRESS, ADDRESS);
    private static final MethodHandle CustomFactor_error = Lib.down(
            "CustomFactor_error", JAVA_DOUBLE, ADDRESS, ADDRESS);

    /** @param p pointer to the factor itself, not the shared_ptr. */
    CustomFactor(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<CustomFactor> newCustomFactor(
            SharedNoiseModel noiseModel,
            KeyVector keys,
            CustomErrorFunction errorFunction) throws Throwable {
        MemorySegment errorFunctionPtr = Lib.linker.upcallStub(
                CustomErrorFunction.f.bindTo(errorFunction),
                FunctionDescriptor.of(ADDRESS, ADDRESS, ADDRESS, ADDRESS),
                Lib.arena);
        MemorySegment sharedPtrPtr = (MemorySegment) CustomFactor.invokeExact(
                noiseModel.ptr, keys.ptr, errorFunctionPtr);
        return new shared_ptr<>(sharedPtrPtr, CustomFactor::new);
    }

    public KeyVector keys() throws Throwable {
        return new KeyVector((MemorySegment) CustomFactor_keys.invokeExact(ptr));
    }

    public double error(Values v) throws Throwable {
        return (double) CustomFactor_error.invokeExact(ptr, v.ptr);
    }

}
