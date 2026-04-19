package gtsam;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.MethodType;

/** See gtsam/nonlinear/CustomFactor.h */
public interface CustomErrorFunction {
    static final MethodHandle f = errorFunctionHandle();

    static MethodHandle errorFunctionHandle() {
        try {
            return MethodHandles.lookup().findVirtual(
                    CustomErrorFunction.class,
                    "bind",
                    MethodType.methodType(
                            MemorySegment.class,
                            MemorySegment.class,
                            MemorySegment.class,
                            MemorySegment.class));
        } catch (NoSuchMethodException | IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }

    Vector apply(CustomFactor factor, Values v, JacobianVector H) throws Throwable;

    /**
     * Arguments are locally-scoped pointers to the parameters of
     * the GTSAM C++ error function, and as such, they are not
     * owned by this function.
     * 
     * @param factor   Pointer to "this" CustomFactor
     * @param values   Pointer to the factor values
     * @param jacobian Nullable pointer to jacobians
     * @return Pointer to owned error vector, should be copied.
     * @throws Throwable
     */
    default MemorySegment bind(
            MemorySegment factor,
            MemorySegment values,
            MemorySegment jacobian) throws Throwable {
        Vector v = apply(
                // Must be observer.
                new CustomFactor(factor),
                // Must be observer.
                Values.observed(values),
                // Must be observer.
                JacobianVector.fromPointer(jacobian));
        return v.ptr;
    }
}
