package gtsam;

import org.junit.jupiter.api.Test;

public class ValuesExtraTest {
    @Test
    void testKeys() {
        Values values = new Values();
        values.insert(Key.X(0), new Point3(1, 2, 3));
        values.insert(Key.Y(1), new Point2(4, 5));
        KeyVector keys = values.keys();
        for (int i = 0; i < keys.size(); ++i) {
            Key key = keys.at(i);
            Symbol s = new Symbol(key);
            switch (s.chr()) {
                case 'x' -> {
                    Point3 value = values.atPoint3(key);
                    System.out.printf("%d %s [%5.2f %5.2f %5.2f]\n", i, s, value.x(), value.y(), value.z());
                }
                case 'y' -> {
                    Point2 value = values.atPoint2(key);
                    System.out.printf("%d %s [%5.2f %5.2f]\n", i, s, value.x(), value.y());
                }
                default -> {
                    System.out.printf("%d %s\n", i, s);
                }
            }
        }
    }
}
