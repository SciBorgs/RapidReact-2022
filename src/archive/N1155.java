package frc.robot;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;

/**
 * A class representing the number 1155.
 */
public final class N1155 extends Num implements Nat<N1155> {
    private static final int int_1155 = 1155;
    private static final Integer obj_1155 = Integer.valueOf(int_1155);

    private N1155() {}

    /**
     * The integer this class represents.
     * 
     * @return The literal number 1155.
     */
    @Override
    public int getNum() {
        return obj_1155;
    }

    /**
     * The singleton instance of this class.
     */
    public static final N1155 instance = new N1155();
}
