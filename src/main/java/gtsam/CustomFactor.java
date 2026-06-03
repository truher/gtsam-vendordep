package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class CustomFactor extends NonlinearFactor {
    public enum FF {
        CustomFactor(ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        CustomFactor_keys(ADDRESS, ADDRESS),
        CustomFactor_error(JAVA_DOUBLE, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    /** @param p pointer to the factor itself, not the shared_ptr. */
    CustomFactor(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<CustomFactor> newCustomFactor(
            shared_ptr<? extends gtsam.noiseModel.Base> noiseModel,
            KeyVector keys,
            CustomErrorFunction errorFunction) throws Throwable {
        MethodHandle bindHandle = CustomErrorFunction.f.bindTo(errorFunction);
        MemorySegment errorFunctionPtr = Lib.linker.upcallStub(
                bindHandle,
                FunctionDescriptor.of(ADDRESS, ADDRESS, ADDRESS, ADDRESS),
                Lib.arena);
        MemorySegment sharedPtrPtr = (MemorySegment) FF.CustomFactor.h.invokeExact(
                noiseModel.ptr, keys.ptr, errorFunctionPtr);
        return new shared_ptr<>(sharedPtrPtr, CustomFactor::new);
    }

    public KeyVector keys() throws Throwable {
        return new KeyVector((MemorySegment) FF.CustomFactor_keys.h.invokeExact(ptr));
    }

    public double error(Values v) throws Throwable {
        return (double) FF.CustomFactor_error.h.invokeExact(ptr, v.ptr);
    }

}
