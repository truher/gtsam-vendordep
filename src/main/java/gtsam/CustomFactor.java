package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class CustomFactor extends NonlinearFactor {
    public enum FF {
        CustomFactor(ADDRESS, ADDRESS, ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    /** @param p pointer to the factor itself, not the shared_ptr. */
    CustomFactor(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<CustomFactor> newCustomFactor(
            shared_ptr<? extends gtsam.noiseModel.Base> noiseModel,
            KeyVector keys,
            CustomErrorFunction errorFunction) {
        MethodHandle bindHandle = CustomErrorFunction.f.bindTo(errorFunction);
        MemorySegment errorFunctionPtr = Lib.linker.upcallStub(
                bindHandle,
                FunctionDescriptor.of(ADDRESS, ADDRESS, ADDRESS, ADDRESS),
                Lib.arena);
        MemorySegment sharedPtrPtr = (MemorySegment) FF.CustomFactor.invoke(
                noiseModel.ptr, keys.ptr, errorFunctionPtr);
        return new shared_ptr<>(sharedPtrPtr, CustomFactor::new);
    }

}
