package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Cal3DS2 extends ForeignObject implements Manifold<Cal3DS2, Vector9> {
    public enum FF {
        Cal3DS2(ADDRESS,
                JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
                JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
                JAVA_DOUBLE, JAVA_DOUBLE),
        Cal3DS2_delete(null, ADDRESS),
        Cal3DS2_localCoordinates(ADDRESS, ADDRESS, ADDRESS),
        Cal3DS2_retract(ADDRESS, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public static class Traits implements Manifold.Traits<Cal3DS2, Vector9> {

    }

    public static final Traits traits = new Traits();

    @Override
    public Traits traits() {
        return traits;
    }

    @Override
    public Vector9 dxZero() throws Throwable {
        return new Vector9(0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    public Cal3DS2(MemorySegment p) {
        super(p, FF.Cal3DS2_delete.h);
    }

    public Cal3DS2(double fx, double fy, //
            double s, double u0, double v0, //
            double k1, double k2, //
            double p1, double p2, //
            double tol) throws Throwable {
        this((MemorySegment) FF.Cal3DS2.h.invokeExact(fx, fy, s, u0, v0, k1, k2, p1, p2, tol));
    }

    public Cal3DS2(double fx, double fy, //
            double s, double u0, double v0, //
            double k1, double k2) throws Throwable {
        this(fx, fy, s, u0, v0, k1, k2, 0.0, 0.0, 1e-5);
    }

    @Override
    public Vector9 localCoordinates(Cal3DS2 g) throws Throwable {
        return new Vector9(
                (MemorySegment) FF.Cal3DS2_localCoordinates.h.invokeExact(ptr, g.ptr));
    }

    @Override
    public int dimension() {
        return 9;
    }

    @Override
    public Cal3DS2 retract(Vector9 v) throws Throwable {
        return new Cal3DS2((MemorySegment) FF.Cal3DS2_retract.h.invokeExact(ptr, v.ptr()));
    }
}
