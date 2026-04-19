package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

/**
 * See gtsam/base/Lie.h
 * 
 * typedef Eigen::Matrix<double, N, 1> TangentVector;
 * for Pose2 this means Matrix<double, 3, 1>
 */
public class TangentVector extends ForeignObject {
    private static final MethodHandle TangentVector_delete = Lib.downVoid(
            "TangentVector_delete", ADDRESS);
    private static final MethodHandle TangentVector_unaryMinus = Lib.down(
            "TangentVector_unaryMinus", ADDRESS, ADDRESS);

    public TangentVector(MemorySegment p) {
        super(p, TangentVector_delete);
    }

    public TangentVector unaryMinus() throws Throwable {
        return new TangentVector(
                (MemorySegment) TangentVector_unaryMinus.invokeExact(ptr));
    }

}
