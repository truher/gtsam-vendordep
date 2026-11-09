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
        PriorFactorPoint2(ADDRESS, JAVA_LONG, ADDRESS, ADDRESS),
        PriorFactorPoint3(ADDRESS, JAVA_LONG, ADDRESS, ADDRESS),
        PriorFactorPose2(ADDRESS, JAVA_LONG, ADDRESS, ADDRESS),
        PriorFactorPose3(ADDRESS, JAVA_LONG, ADDRESS, ADDRESS),
        PriorFactorVector(ADDRESS, JAVA_LONG, ADDRESS, ADDRESS),
        PriorFactorVector3(ADDRESS, JAVA_LONG, ADDRESS, ADDRESS),
        PriorFactorCal3DS2(ADDRESS, JAVA_LONG, ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    /** @param p pointer to the factor itself, not the shared_ptr. */
    private PriorFactor(MemorySegment p) {
        super(p);
    }

    /**
     * @throws IllegalArgumentException if the model dimension is not 1
     */
    public static shared_ptr<PriorFactor<Double>> PriorFactorDouble(
            Key key,
            double prior,
            shared_ptr<? extends gtsam.noiseModel.Base> model) {
        if (model.get().dim() != 1)
            throw new IllegalArgumentException();
        return new shared_ptr<>(
                (MemorySegment) FF.PriorFactorDouble.invoke(key.j, prior, model.ptr),
                PriorFactor::new);
    }

    /**
     * @param prior is copied, ok to delete.
     * @throws IllegalArgumentException if the model dimension is not 2
     */
    public static shared_ptr<PriorFactor<Point2>> PriorFactorPoint2(
            Key key,
            Point2 prior,
            shared_ptr<? extends gtsam.noiseModel.Base> model) {
        if (model.get().dim() != 2)
            throw new IllegalArgumentException();
        return new shared_ptr<>(
                (MemorySegment) FF.PriorFactorPoint2.invoke(key.j, prior.ptr, model.ptr),
                PriorFactor::new);
    }

    /**
     * @param prior is copied, ok to delete.
     * @throws IllegalArgumentException if the model dimension is not 3
     */
    public static shared_ptr<PriorFactor<Point3>> PriorFactorPoint3(
            Key key,
            Point3 prior,
            shared_ptr<? extends gtsam.noiseModel.Base> model) {
        if (model.get().dim() != 3)
            throw new IllegalArgumentException();
        return new shared_ptr<>(
                (MemorySegment) FF.PriorFactorPoint3.invoke(key.j, prior.ptr, model.ptr),
                PriorFactor::new);
    }

    /**
     * @param prior is copied, ok to delete.
     * @throws IllegalArgumentException if the model dimension is not 3
     */
    public static shared_ptr<PriorFactor<Pose2>> PriorFactorPose2(
            Key key,
            Pose2 prior,
            shared_ptr<? extends gtsam.noiseModel.Base> model) {
        if (model.get().dim() != 3)
            throw new IllegalArgumentException();
        return new shared_ptr<>(
                (MemorySegment) FF.PriorFactorPose2.invoke(key.j, prior.ptr, model.ptr),
                PriorFactor::new);
    }

    /**
     * @param prior is copied, ok to delete.
     * @throws IllegalArgumentException if the model dimension is not 6
     */
    public static shared_ptr<PriorFactor<Pose3>> PriorFactorPose3(
            Key key,
            Pose3 prior,
            shared_ptr<? extends gtsam.noiseModel.Base> model) {
        if (model.get().dim() != 6)
            throw new IllegalArgumentException();
        return new shared_ptr<>(
                (MemorySegment) FF.PriorFactorPose3.invoke(key.j, prior.ptr, model.ptr),
                PriorFactor::new);
    }

    /**
     * @throws IllegalArgumentException if the model dimension doesn't match the
     *                                  vector dimension.
     */
    public static shared_ptr<PriorFactor<Vector>> PriorFactorVector(
            Key key,
            Vector prior,
            shared_ptr<? extends gtsam.noiseModel.Base> model) {
        if (model.get().dim() != prior.dimension())
            throw new IllegalArgumentException();
        return new shared_ptr<>(
                (MemorySegment) FF.PriorFactorVector.invoke(key.j, prior.ptr, model.ptr),
                PriorFactor::new);
    }

    /**
     * @throws IllegalArgumentException if the model dimension is not 3
     */
    public static shared_ptr<PriorFactor<Vector3>> PriorFactorVector3(
            Key key,
            Vector3 prior,
            shared_ptr<? extends gtsam.noiseModel.Base> model) {
        if (model.get().dim() != 3)
            throw new IllegalArgumentException();
        return new shared_ptr<>(
                (MemorySegment) FF.PriorFactorVector3.invoke(key.j, prior.ptr, model.ptr),
                PriorFactor::new);
    }

    /**
     * @param prior is copied, ok to delete.
     * @throws IllegalArgumentException if the model dimension is not 9
     */
    public static shared_ptr<PriorFactor<Cal3DS2>> PriorFactorCal3DS2(
            Key key,
            Cal3DS2 prior,
            shared_ptr<? extends gtsam.noiseModel.Base> model) {
        if (model.get().dim() != 9)
            throw new IllegalArgumentException();
        return new shared_ptr<>(
                (MemorySegment) FF.PriorFactorCal3DS2.invoke(key.j, prior.ptr, model.ptr),
                PriorFactor::new);
    }
}
