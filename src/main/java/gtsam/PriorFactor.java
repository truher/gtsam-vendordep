package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

/**
 * The generic works here because there are no methods; this
 * only exists to add things to the graph.
 * TODO: add methods, make separate classes.
 */
public class PriorFactor<T> extends NoiseModelFactor {
    public enum FF {
        PriorFactorDouble(ADDRESS, JAVA_LONG, JAVA_DOUBLE, ADDRESS),
        PriorFactorPose2(ADDRESS, JAVA_LONG, ADDRESS, ADDRESS),
        PriorFactorPose3(ADDRESS, JAVA_LONG, ADDRESS, ADDRESS),
        PriorFactorCal3DS2(ADDRESS, JAVA_LONG, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    /** @param p pointer to the factor itself, not the shared_ptr. */
    private PriorFactor(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<PriorFactor<Double>> PriorFactorDouble(
            Key poseKey,
            double prior,
            shared_ptr<? extends gtsam.noiseModel.Base> model) throws Throwable {
        return new shared_ptr<>(
                (MemorySegment) FF.PriorFactorDouble.h.invokeExact(poseKey.j, prior, model.ptr),
                PriorFactor::new);
    }

    /** @param prior is copied, ok to delete. */
    public static shared_ptr<PriorFactor<Pose2>> PriorFactorPose2(
            Key poseKey,
            Pose2 prior,
            shared_ptr<? extends gtsam.noiseModel.Base> model) throws Throwable {
        return new shared_ptr<>(
                (MemorySegment) FF.PriorFactorPose2.h.invokeExact(poseKey.j, prior.ptr, model.ptr),
                PriorFactor::new);
    }

    /** @param prior is copied, ok to delete. */
    public static shared_ptr<PriorFactor<Pose3>> PriorFactorPose3(
            Key key,
            Pose3 prior,
            shared_ptr<? extends gtsam.noiseModel.Base> model) throws Throwable {
        return new shared_ptr<>(
                (MemorySegment) FF.PriorFactorPose3.h.invokeExact(key.j, prior.ptr, model.ptr),
                PriorFactor::new);

    }

    /** @param prior is copied, ok to delete. */
    public static shared_ptr<PriorFactor<Cal3DS2>> PriorFactorCal3DS2(
            Key key,
            Cal3DS2 prior,
            shared_ptr<? extends gtsam.noiseModel.Base> model) throws Throwable {
        return new shared_ptr<>(
                (MemorySegment) FF.PriorFactorCal3DS2.h.invokeExact(key.j, prior.ptr, model.ptr),
                PriorFactor::new);
    }
}
