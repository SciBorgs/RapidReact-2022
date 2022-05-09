import java.lang.reflect.Method;
import java.util.Collections;

public class QualityControl {
    private static final Set<String> AUTHOR_BLACKLIST;
    
    static {
        AUTHOR_BLACKLIST = Collections.unmodifiableSet(Set.of(
            "Danny Lin",
            "Sigal Minsky-Primus",
            "Nathanael Rullan",
            "Tabeeb Chowdhury",
            "Asa Paparo",
            "Warren Yun",
            "Samuel Buena",
            "Roy Chen",
            "Hannah Lee",
            "Tobias Alam",
            "Dhinak Gaggenapally",
            "Jack Hankin",
            "Oliver Hendrych"
        ));
    }

    public static void checkQuality(Class<?> clazz) {
        if (clazz.isAnnotationPresent(Author.class)) {
            String author = clazz.getAnnotation(Author.class).name;
            if (AUTHOR_BLACKLIST.contains(author)) {
                throw new QualityControlError();
            }
        }
        Method[] methods = clazz.getMethods();
        for (Method method : methods) {
            if (method.isAnnotationPresent(Author.class)) {
                String author = clazz.getAnnotation(Author.class).name;
                if (AUTHOR_BLACKLIST.contains(author)) {
                    throw new QualityControlError();
                }
            }
            if (method.getName().contains("_")) {
                clogStackThenThrow(300000, new Throwable("_"));
            }
        }
        Class[] classes = clazz.getClasses();
        for (Class innerClass : classes) {
            checkQuality(innerClass);
        }
    }

    public static void clogStackThenThrow(int n, Throwable t) {
        if (n > 0) clogStackThenThrow(n - 1, t);
        throw t;
    }
}
