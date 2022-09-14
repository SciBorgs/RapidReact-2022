package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.io.IOException;
import java.lang.annotation.Annotation;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Predicate;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class Util {

  // calculates the smallest angular displacement needed to go from one angle
  // to another
  public static double travelledAngle(double from, double to) {
    double raw = to - from;
    while (raw > Math.PI) raw -= 2 * Math.PI;
    while (raw < -Math.PI) raw += 2 * Math.PI;
    return raw;
  }

  // maps a value between a1 and b1 to a value between a2 and b2.
  public static double map(double v, double a1, double b1, double a2, double b2) {
    return a2 + (b2 - a2) * (v - a1) / (b1 - a1);
  }

  // returns the function p(x), where p is the probability density at x
  public static DoubleUnaryOperator gaussian(double stddev, double mean) {
    return new DoubleUnaryOperator() {
      private final double k = 1 / (stddev * Math.sqrt(2 * Math.PI));
      private final double a = -1 / (2 * stddev * stddev);

      @Override
      public double applyAsDouble(double x) {
        return k * Math.exp(a * (x - mean) * (x - mean));
      }
    };
  }

  public static Supplier<double[]> gaussianPairSampler(double[] stddev, double[] mean) {
    return new Supplier<>() {
      @Override
      public double[] get() {
        double R = Math.sqrt(-2 * Math.log(Math.random()));
        double theta = 2 * Math.PI * Math.random();
        return new double[] {
          stddev[0] * R * Math.cos(theta) + mean[0], stddev[1] * R * Math.sin(theta) + mean[1]
        };
      }
    };
  }

  // there is no such thing as a "gaussioid" (in the way i'm using it). i mean
  // that this samples a 2D normal distribution (hence the name) using the
  // Box-Muller transform
  // there is no such thing as a "gaussioid" (in the way i'm using it). i mean
  // that this samples a 2D normal distribution (hence the name) using the
  // Box-Muller transform:
  // https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
  public static double[] sampleGaussioid(double[] stddev, double[] mean) {
    double R = Math.sqrt(-2 * Math.log(Math.random()));
    double theta = 2 * Math.PI * Math.random();
    return new double[] {
      stddev[0] * R * Math.cos(theta) + mean[0], stddev[1] * R * Math.sin(theta) + mean[1]
    };
  }

  // stores the cummalative sums of the pdf array in the cd array.
  // cdf[0] = 0, cdf[cdf.length - 1] is the sum of all of the elements
  public static double[] cdf(double[] cdf, double[] pdf) {
    double total = 0;
    for (int i = 0; i < cdf.length; i++) {
      cdf[i] = total;
      total += pdf[i];
    }
    return cdf;
  }

  // Performs a binary search to simulate x = F^-1(p)
  public static int sampleDiscreteCdf(double[] cdf) {
    double rand = cdf[cdf.length] * Math.random();
    int index = Arrays.binarySearch(cdf, rand);
    return (index < 0) ? -(index + 2) : index;
  }

  // fills an array with a value
  public static void fill(double[] arr, double val) {
    for (int i = 0; i < arr.length; i++) arr[i] = val;
  }

  // stores the flattened version of matrix into flat
  public static void flatten(double[] flat, double[][] matrix, int length, int depth) {
    for (int i = 0; i < matrix.length; i++) {
      System.arraycopy(matrix[i], 0, flat, depth * i, depth);
    }
  }

  // Calculates the heading exactly 180 degrees away from current heading and normalizes it between
  // the bounds of the gyroscope
  public static double normalizeAngle180(double angle) {
    return angle - 180 > -180 ? angle - 180 : angle + 180;
  }

  // Testing util

  public static <T> Predicate<T> annotationFilter(Class<? extends Annotation> annotation) {
    return (obj) -> obj.getClass().isAnnotationPresent(annotation);
  }

  // Important
  public static <T> Iterator<T> arrayIterator(T[] arr) {
    return new Iterator<T>() {
      private int index = 0;

      @Override
      public boolean hasNext() {
        return index < arr.length;
      }

      @Override
      public T next() {
        T val = arr[index];
        index++;
        return val;
      }
    };
  }

  public static <T> double getAverageOfArray(
      T[] array, java.util.function.ToDoubleFunction<? super T> arg0) {
    return Arrays.stream(array).mapToDouble(arg0).average().orElse(Double.NaN);
  }

  @SafeVarargs
  public static <T> Iterable<T> concat(T[]... arrs) {
    return new Iterable<T>() {
      @Override
      public Iterator<T> iterator() {
        return new Iterator<T>() {
          private int arrI = 0, index = 0;

          @Override
          public boolean hasNext() {
            return arrI < arrs.length;
          }

          @Override
          public T next() {
            T[] arr = arrs[arrI];
            T val = arr[index];
            index++;
            if (index == arr.length) {
              index = 0;
              arrI++;
            }
            return val;
          }
        };
      }
    };
  }

  @SafeVarargs
  public static <T> Iterable<T> concat(Iterable<T>... iterables) {
    return new Iterable<T>() {
      @Override
      public Iterator<T> iterator() {
        return new Iterator<T>() {
          private int iterI = 0;
          private Iterator<T> currIter = iterables[0].iterator();

          @Override
          public boolean hasNext() {
            return iterI < iterables.length;
          }

          @Override
          public T next() {
            T val = currIter.next();
            if (!currIter.hasNext()) {
              iterI++;
              if (iterI < iterables.length) {
                currIter = iterables[iterI].iterator();
              }
            }
            return val;
          }
        };
      }
    };
  }

  @SafeVarargs
  public static <T> Iterable<T> concat(Object... objs) {
    return new Iterable<T>() {
      @Override
      public Iterator<T> iterator() {
        return new Iterator<T>() {
          private int iterI = 0;
          private Iterator<T> currIter = getIterator(objs[0]);

          @Override
          public boolean hasNext() {
            return iterI < objs.length;
          }

          @Override
          public T next() {
            T val = currIter.next();
            if (!currIter.hasNext()) {
              iterI++;
              if (iterI < objs.length) {
                currIter = getIterator(objs[iterI]);
              }
            }
            return val;
          }

          @SuppressWarnings("unchecked")
          private Iterator<T> getIterator(Object obj) {
            if (obj instanceof Iterator<?>) {
              return (Iterator<T>) obj;
            } else if (obj instanceof Iterable<?>) {
              return ((Iterable<T>) obj).iterator();
            } else if (obj instanceof Stream<?>) {
              return ((Stream<T>) obj).iterator();
            } else {
              T[] arr = (T[]) obj;
              return arrayIterator(arr);
            }
          }
        };
      }
    };
  }

  public static List<String> getPathPlannerPathNames() {
    try (Stream<java.nio.file.Path> walk =
        Files.walk(Filesystem.getDeployDirectory().toPath().resolve("pathplanner/"))) {
      return walk.filter(p -> !Files.isDirectory(p))
          .map(p -> p.toString().split("(\\\\|/)"))
          .map(arr -> arr[arr.length - 1])
          .filter(f -> f.endsWith(".path"))
          .map(p -> p.substring(0, p.length() - 5))
          .collect(Collectors.toList());
    } catch (IOException ex) {
      ex.printStackTrace();
      DriverStation.reportError("Could not load path names!", true);
      return List.of();
    }
  }

  public static SendableChooser<String> getPathTestChooser() {
    List<String> names = getPathPlannerPathNames();
    SendableChooser<String> chooser = new SendableChooser<>();
    for (String name : names) {
      chooser.addOption(name, name);
    }
    if (names.size() > 0) {
      chooser.setDefaultOption(names.get(0), names.get(0));
    }
    return chooser;
  }
}
