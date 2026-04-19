package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Matrix3 extends ForeignObject {
    private static final MethodHandle Matrix3 = Lib.down(
            "Matrix3", ADDRESS,
            JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
            JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
            JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE);
    private static final MethodHandle Matrix3_delete = Lib.downVoid(
            "Matrix3_delete", ADDRESS);
    private static final MethodHandle Matrix3_unaryMinus = Lib.down(
            "Matrix3_unaryMinus", ADDRESS, ADDRESS);
    private static final MethodHandle Matrix3_identity = Lib.down(
            "Matrix3_identity", ADDRESS);

    public Matrix3(MemorySegment p) {
        super(p, Matrix3_delete);
    }

    public Matrix3( //
            double R11, double R12, double R13, //
            double R21, double R22, double R23, //
            double R31, double R32, double R33) throws Throwable {
        this((MemorySegment) Matrix3.invokeExact(//
                R11, R12, R13, //
                R21, R22, R23, //
                R31, R32, R33));
    }

    public Matrix3 unaryMinus() throws Throwable {
        return new Matrix3(
                (MemorySegment) Matrix3_unaryMinus.invokeExact(ptr));
    }

    public static Matrix3 identity() throws Throwable {
        return new Matrix3((MemorySegment) Matrix3_identity.invokeExact());
    }

}
