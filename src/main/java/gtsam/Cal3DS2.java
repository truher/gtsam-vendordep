package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Cal3DS2 extends ForeignObject implements Manifold<Cal3DS2, Vector9> {
    private static final MethodHandle Cal3DS2 = Lib.down(
            "Cal3DS2", ADDRESS,
            JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
            JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
            JAVA_DOUBLE, JAVA_DOUBLE);
    private static final MethodHandle Cal3DS2_delete = Lib.downVoid(
            "Cal3DS2_delete", ADDRESS);
    private static final MethodHandle Cal3DS2_localCoordinates = Lib.down(
            "Cal3DS2_localCoordinates", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Cal3DS2_retract = Lib.down(
            "Cal3DS2_retract", ADDRESS, ADDRESS, ADDRESS);

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
    public Vector9 localCoordinates(Cal3DS2 g) throws Throwable {
        return new Vector9(
                (MemorySegment) Cal3DS2_localCoordinates.invokeExact(ptr, g.ptr));
    }

    @Override
    public int dimension() {
        return 9;
    }

    @Override
    public Cal3DS2 retract(Vector9 v) throws Throwable {
        return new Cal3DS2((MemorySegment) Cal3DS2_retract.invokeExact(ptr, v.ptr()));
    }
}
