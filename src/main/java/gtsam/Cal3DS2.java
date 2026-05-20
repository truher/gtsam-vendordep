package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Cal3DS2 extends ForeignObject implements LieGroup<Cal3DS2> {
    private static final MethodHandle Cal3DS2 = Lib.down(
            "Cal3DS2", ADDRESS,
            JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
            JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
            JAVA_DOUBLE, JAVA_DOUBLE);
    private static final MethodHandle Cal3DS2_delete = Lib.downVoid(
            "Cal3DS2_delete", ADDRESS);
    private static final MethodHandle Cal3DS2_retract = Lib.down(
            "Cal3DS2_retract", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Cal3DS2_print = Lib.downVoid(
            "Cal3DS2_print", ADDRESS);
    private static final MethodHandle Cal3DS2_equals = Lib.down(
            "Cal3DS2_equals", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);

    public Cal3DS2(MemorySegment p) {
        super(p, Cal3DS2_delete);
    }

    public Cal3DS2(double fx, double fy, //
            double s, double u0, double v0, //
            double k1, double k2, //
            double p1, double p2, //
            double tol) throws Throwable {
        this((MemorySegment) Cal3DS2.invokeExact(fx, fy, s, u0, v0, k1, k2, p1, p2, tol));
    }

    public Cal3DS2(double fx, double fy, //
            double s, double u0, double v0, //
            double k1, double k2) throws Throwable {
        this(fx, fy, s, u0, v0, k1, k2, 0.0, 0.0, 1e-5);
    }

    @Override
    public int dimension() {
        return 9;
    }

    @Override
    public Cal3DS2 retract(VectorSpace<?> v) throws Throwable {
        return new Cal3DS2((MemorySegment) Cal3DS2_retract.invokeExact(ptr, v.ptr()));
    }

    @Override
    public void print() throws Throwable {
        Cal3DS2_print.invokeExact(ptr);
    }

    public boolean equals(Cal3DS2 other, double tol) throws Throwable {
        return (boolean) Cal3DS2_equals.invokeExact(ptr, other.ptr, tol);
    }
}
