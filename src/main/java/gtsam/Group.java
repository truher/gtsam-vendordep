package gtsam;

/**
 * See gtsam/base/Group.h.
 * 
 * A group has an identity element, one associative operation, a.compose(b),
 * and an inverse operation.
 * 
 * GTSAM also defines a "between" operation, which is a.inverse().compose(b).
 * 
 * https://en.wikipedia.org/wiki/Group_(mathematics)
 */
public interface Group<T extends Group<T>> {
    public interface Companion<T extends Group<T>> {
        T Identity() throws Throwable;
    }

    Companion<T> companion();

    T compose(T h) throws Throwable;

    T compose(T h, Matrix H1, Matrix H2) throws Throwable;

    T between(T h) throws Throwable;

    T between(T h, Matrix H1, Matrix H2) throws Throwable;

    T inverse() throws Throwable;

    T inverse(Matrix H) throws Throwable;
}
