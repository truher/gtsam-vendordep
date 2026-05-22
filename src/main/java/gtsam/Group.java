package gtsam;

/**
 * See gtsam/base/Group.h.
 * 
 * TODO: add Jacobians.
 */
public interface Group<T extends Group<T>> {
    public interface Traits<T extends Group<T>> {
        T Identity() throws Throwable;

        default T Compose(T g, T h) throws Throwable {
            return g.compose(h);
        }

        default T Between(T g, T h) throws Throwable {
            return g.between(h);
        }

        default T Inverse(T g) throws Throwable {
            return g.inverse();
        }
    }

    Traits<T> traits();

    T compose(T h) throws Throwable;

    T between(T h) throws Throwable;

    T inverse() throws Throwable;
}
