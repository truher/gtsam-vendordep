package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Cal3DS2 extends ForeignObject {
    private static final MethodHandle Cal3DS2 = Lib.down(
            "Cal3DS2", ADDRESS,
            JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
            JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
            JAVA_DOUBLE, JAVA_DOUBLE);
    private static final MethodHandle Cal3DS2_delete = Lib.downVoid(
            "Cal3DS2_delete", ADDRESS);

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
}
