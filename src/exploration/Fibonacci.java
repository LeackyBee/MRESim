package exploration;

import java.util.ArrayList;
import java.util.List;

public class Fibonacci {

    private static final List<Integer> fibNumbers = new ArrayList<>();

    static{
        fibNumbers.add(1);
        fibNumbers.add(1);
    }

    public static int fib(int x){
        if(x > fibNumbers.size()-1){
            int f1 = fib(x-1);
            int f2 = fib(x-2);
            fibNumbers.add(f1+f2);
        }

        return fibNumbers.get(x);
    }
}
