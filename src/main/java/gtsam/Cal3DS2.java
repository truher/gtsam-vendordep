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
        Cal3DS2_retract(ADDRESS, ADDRESS, ADDRESS),
        Cal3DS2_uncalibrate(ADDRESS, ADDRESS, ADDRESS),
        Cal3DS2_uncalibrateH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Cal3DS2_calibrate(ADDRESS, ADDRESS, ADDRESS),
        Cal3DS2_calibrateH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Cal3DS2_D2d_calibration(ADDRESS, ADDRESS, ADDRESS),
        Cal3DS2_D2d_intrinsic(ADDRESS, ADDRESS, ADDRESS),
        Cal3DS2_k(ADDRESS, ADDRESS),
        Cal3DS2_K(ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    @Override
    public Vector9 dxZero() throws Throwable {
        return new Vector9(0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    public static int Dim() {
        return 9;
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
            double k1, double k2, //
            double p1, double p2) throws Throwable {
        this((MemorySegment) FF.Cal3DS2.h.invokeExact(fx, fy, s, u0, v0, k1, k2, p1, p2, 1e-5));
    }

    public Cal3DS2(double fx, double fy, //
            double s, double u0, double v0, //
            double k1, double k2) throws Throwable {
        this(fx, fy, s, u0, v0, k1, k2, 0.0, 0.0, 1e-5);
    }

    @Override
    public Vector9 local(Cal3DS2 g) throws Throwable {
        return new Vector9(
                (MemorySegment) FF.Cal3DS2_localCoordinates.h.invokeExact(ptr, g.ptr));
    }

    @Override
    public Vector9 local(Cal3DS2 g, Matrix H1, Matrix H2) throws Throwable {
        throw new UnsupportedOperationException();
    }

    @Override
    public int dimension() {
        return Cal3DS2.Dim();
    }

    public int dim() {
        return Cal3DS2.Dim();
    }

    @Override
    public Cal3DS2 retract(Vector9 v) throws Throwable {
        return new Cal3DS2((MemorySegment) FF.Cal3DS2_retract.h.invokeExact(ptr, v.ptr));
    }

    @Override
    public Cal3DS2 retract(Vector9 v, Matrix H1, Matrix H2) throws Throwable {
        throw new UnsupportedOperationException();
    }

    public Point2 uncalibrate(Point2 p) throws Throwable {
        return new Point2((MemorySegment) FF.Cal3DS2_uncalibrate.h.invokeExact(ptr, p.ptr));
    }

    public Point2 uncalibrate(Point2 p, Matrix H1, Matrix H2) throws Throwable {
        return new Point2((MemorySegment) FF.Cal3DS2_uncalibrateH.h.invokeExact(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public Point2 calibrate(Point2 p) throws Throwable {
        return new Point2((MemorySegment) FF.Cal3DS2_calibrate.h.invokeExact(ptr, p.ptr));
    }

    public Point2 calibrate(Point2 p, Matrix H1, Matrix H2) throws Throwable {
        return new Point2((MemorySegment) FF.Cal3DS2_calibrateH.h.invokeExact(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public Matrix D2d_calibration(Point2 p) throws Throwable {
        return new Matrix((MemorySegment) FF.Cal3DS2_D2d_calibration.h.invokeExact(ptr, p.ptr));
    }

    public Matrix D2d_intrinsic(Point2 p) throws Throwable {
        return new Matrix((MemorySegment) FF.Cal3DS2_D2d_intrinsic.h.invokeExact(ptr, p.ptr));
    }

    public Vector4 k() throws Throwable {
        return new Vector4((MemorySegment) FF.Cal3DS2_k.h.invokeExact(ptr));
    }

    public Matrix3 K() throws Throwable {
        return new Matrix3((MemorySegment) FF.Cal3DS2_K.h.invokeExact(ptr));
    }
}
