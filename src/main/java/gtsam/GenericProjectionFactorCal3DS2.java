package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class GenericProjectionFactorCal3DS2 extends NoiseModelFactor {

    public enum FF {
        GenericProjectionFactorCal3DS2(ADDRESS, ADDRESS, ADDRESS, JAVA_LONG, JAVA_LONG, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    GenericProjectionFactorCal3DS2(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<GenericProjectionFactorCal3DS2>//
            newGenericProjectionFactorCal3DS2(
                    Point2 measured, //
                    shared_ptr<? extends gtsam.noiseModel.Base> model,
                    Key poseKey, //
                    Key pointKey, //
                    shared_ptr<Cal3DS2> K, //
                    Pose3 bTc) throws Throwable {
        return new shared_ptr<>((MemorySegment) FF.GenericProjectionFactorCal3DS2.h.invokeExact(//
                measured.ptr, model.ptr, poseKey.j, pointKey.j, K.ptr, bTc.ptr), //
                GenericProjectionFactorCal3DS2::new);
    }
}
