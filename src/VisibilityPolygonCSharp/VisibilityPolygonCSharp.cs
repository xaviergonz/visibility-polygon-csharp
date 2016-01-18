using System;
using System.Collections.Generic;

namespace VisibilityPolygonCSharp
{
  public static class VisibilityPolygonCSharp
  {
    private const double Epsilon = 0.0000001;


    public static List<PointDouble> Compute(PointDouble position, List<SegmentDouble> segments) {
      if (segments == null) {
        throw new ArgumentNullException(nameof(segments));
      }

      var bounded = new List<SegmentDouble>();
      double minX = position.X;
      double minY = position.Y;
      double maxX = position.X;
      double maxY = position.Y;
      for (var i = 0; i < segments.Count; ++i) {
        for (var j = 0; j < 2; ++j) {
          minX = Math.Min(minX, segments[i][j].X);
          minY = Math.Min(minY, segments[i][j].Y);
          maxX = Math.Max(maxX, segments[i][j].X);
          maxY = Math.Max(maxY, segments[i][j].Y);
        }
        //bounded.push([[segments[i][0][0], segments[i][0][1]], [segments[i][1][0], segments[i][1][1]]]);
        bounded.Add(new SegmentDouble(segments[i][0], segments[i][1]));
      }
      --minX;
      --minY;
      ++maxX;
      ++maxY;

      bounded.Add(new SegmentDouble(new PointDouble(minX, minY), new PointDouble(maxX, minY)));
      bounded.Add(new SegmentDouble(new PointDouble(maxX, minY), new PointDouble(maxX, maxY)));
      bounded.Add(new SegmentDouble(new PointDouble(maxX, maxY), new PointDouble(minX, maxY)));
      bounded.Add(new SegmentDouble(new PointDouble(minX, maxY), new PointDouble(minX, minY)));

      var polygon = new List<PointDouble>();
      SegmentPointAngle[] sorted = SortPoints(position, bounded);
      var map = new int[bounded.Count];
      for (var i = 0; i < map.Length; ++i) {
        map[i] = -1;
      }

      var heap = new List<int>();
      var start = new PointDouble(position.X + 1, position.Y);
      for (var i = 0; i < bounded.Count; ++i) {
        double a1 = Angle(bounded[i][0], position);
        double a2 = Angle(bounded[i][1], position);
        bool active = (a1 > -180.0 && a1 <= 0.0 && a2 <= 180.0 && a2 >= 0.0 && a2 - a1 > 180.0) ||
                      (a2 > -180.0 && a2 <= 0.0 && a1 <= 180.0 && a1 >= 0.0 && a1 - a2 > 180.0);
        if (active) {
          Insert(i, heap, position, bounded, start, map);
        }
      }

      for (var i = 0; i < sorted.Length;) {
        var extend = false;
        var shorten = false;
        int orig = i;
        PointDouble vertex = bounded[sorted[i].SegmentIndex][sorted[i].PointIndex];
        int oldSegment = heap[0];
        do {
          if (map[sorted[i].SegmentIndex] != -1) {
            if (sorted[i].SegmentIndex == oldSegment) {
              extend = true;
              vertex = bounded[sorted[i].SegmentIndex][sorted[i].PointIndex];
            }
            Remove(map[sorted[i].SegmentIndex], heap, position, bounded, vertex, map);
          }
          else {
            Insert(sorted[i].SegmentIndex, heap, position, bounded, vertex, map);
            if (heap[0] != oldSegment) {
              shorten = true;
            }
          }
          ++i;
          if (i >= sorted.Length) {
            break;
          }
        } while (sorted[i].Angle < sorted[orig].Angle + Epsilon);

        if (extend) {
          polygon.Add(vertex);
          PointDouble? cur = IntersectLines(bounded[heap[0]][0], bounded[heap[0]][1], position, vertex);
          if (cur.HasValue && !Equal(cur.Value, vertex)) {
            polygon.Add(cur.Value);
          }
        }
        else if (shorten) {
          PointDouble? add1 = IntersectLines(bounded[oldSegment][0], bounded[oldSegment][1], position, vertex);
          if (add1.HasValue) {
            polygon.Add(add1.Value);
          }
          PointDouble? add2 = IntersectLines(bounded[heap[0]][0], bounded[heap[0]][1], position, vertex);
          if (add2.HasValue) {
            polygon.Add(add2.Value);
          }
        }
      }
      return polygon;
    }

    public static List<PointDouble> ComputeViewport(PointDouble position, List<SegmentDouble> segments, PointDouble viewportMinCorner,
      PointDouble viewportMaxCorner) {
      var brokenSegments = new List<SegmentDouble>();
      var viewport = new List<PointDouble> {
        viewportMinCorner,
        new PointDouble(viewportMaxCorner.X, viewportMinCorner.Y),
        viewportMaxCorner,
        new PointDouble(viewportMinCorner.X, viewportMaxCorner.Y)
      };
      for (var i = 0; i < segments.Count; ++i) {
        if (segments[i][0].X < viewportMinCorner.X && segments[i][1].X < viewportMinCorner.X)
          continue;
        if (segments[i][0].Y < viewportMinCorner.Y && segments[i][1].Y < viewportMinCorner.Y)
          continue;
        if (segments[i][0].X > viewportMaxCorner.X && segments[i][1].X > viewportMaxCorner.X)
          continue;
        if (segments[i][0].Y > viewportMaxCorner.Y && segments[i][1].Y > viewportMaxCorner.Y)
          continue;

        var intersections = new List<PointDouble>();
        for (var j = 0; j < viewport.Count; ++j) {
          int k = j + 1;
          if (k == viewport.Count) {
            k = 0;
          }

          if (DoLineSegmentsIntersect(segments[i][0].X, segments[i][0].Y, segments[i][1].X, segments[i][1].Y,
            viewport[j].X, viewport[j].Y, viewport[k].X, viewport[k].Y)) {
            PointDouble? intersect = IntersectLines(segments[i][0], segments[i][1], viewport[j], viewport[k]);
            if (intersect == null) {
              continue;
            }
            if (Equal(intersect, segments[i][0]) || Equal(intersect, segments[i][1])) {
              continue;
            }
            intersections.Add(intersect.Value);
          }
        }

        //var start = new PointDouble(segments[i][0].X, segments[i][0].Y);
        PointDouble start = segments[i][0];
        while (intersections.Count > 0) {
          var endIndex = 0;
          double endDis = Distance(start, intersections[0]);
          for (var j = 1; j < intersections.Count; ++j) {
            double dis = Distance(start, intersections[j]);
            if (dis < endDis) {
              endDis = dis;
              endIndex = j;
            }
          }
          brokenSegments.Add(new SegmentDouble(start, intersections[endIndex]));
          //start.X = intersections[endIndex].X;
          //start.Y = intersections[endIndex].Y;
          start = intersections[endIndex];
          //intersections.splice(endIndex, 1);
          Splice(intersections, endIndex, 1);
        }
        //brokenSegments.push([start, [segments[i][1][0], segments[i][1][1]]]);
        brokenSegments.Add(new SegmentDouble(start, segments[i][1]));
      }

      var viewportSegments = new List<SegmentDouble>();
      for (var i = 0; i < brokenSegments.Count; ++i) {
        if (InViewport(brokenSegments[i][0], viewportMinCorner, viewportMaxCorner) &&
            InViewport(brokenSegments[i][1], viewportMinCorner, viewportMaxCorner)) {
          viewportSegments.Add(new SegmentDouble(brokenSegments[i][0], brokenSegments[i][1]));
        }
      }
      double eps = Epsilon * 10.0;
      viewportSegments.Add(new SegmentDouble(
        new PointDouble(viewportMinCorner.X - eps, viewportMinCorner.Y - eps),
        new PointDouble(viewportMaxCorner.X + eps, viewportMinCorner.Y - eps)
      ));
      viewportSegments.Add(new SegmentDouble(
        new PointDouble(viewportMaxCorner.X + eps, viewportMinCorner.Y - eps),
        new PointDouble(viewportMaxCorner.X + eps, viewportMaxCorner.Y + eps)
      ));
      viewportSegments.Add(new SegmentDouble(
        new PointDouble(viewportMaxCorner.X + eps, viewportMaxCorner.Y + eps),
        new PointDouble(viewportMinCorner.X - eps, viewportMaxCorner.Y + eps)
      ));
      viewportSegments.Add(new SegmentDouble(
        new PointDouble(viewportMinCorner.X - eps, viewportMaxCorner.Y + eps),
        new PointDouble(viewportMinCorner.X - eps, viewportMinCorner.Y - eps)
      ));
      return Compute(position, viewportSegments);
    }


    public static bool InViewport(PointDouble position, PointDouble viewportMinCorner, PointDouble viewportMaxCorner) {
      if (position.X < viewportMinCorner.X - Epsilon) {
        return false;
      }
      if (position.Y < viewportMinCorner.Y - Epsilon) {
        return false;
      }
      if (position.X > viewportMaxCorner.X + Epsilon) {
        return false;
      }
      if (position.Y > viewportMaxCorner.Y + Epsilon) {
        return false;
      }
      return true;
    }


    private static PointDouble? IntersectLines(PointDouble a1, PointDouble a2, PointDouble b1, PointDouble b2) {
      /*
      double uaT = (b2.X - b1.X) * (a1.Y - b1.Y) - (b2.Y - b1.Y) * (a1.X - b1.X);
      //var ubT = (a2.X - a1.X) * (a1.Y - b1.Y) - (a2.Y - a1.Y) * (a1.X - b1.X);
      double uB = (b2.Y - b1.Y) * (a2.X - a1.X) - (b2.X - b1.X) * (a2.Y - a1.Y);

      // ReSharper disable once CompareOfFloatsByEqualityOperator
      if (uB != 0.0) {
        double ua = uaT / uB;
        //var ub = ubT / uB;
        return new PointDouble(a1.X - ua * (a1.X - a2.X), a1.Y - ua * (a1.Y - a2.Y));
      }
      return null;
      */

      // optimized version
      double dbx = b2.X - b1.X, dby = b2.Y - b1.Y, dax = a2.X - a1.X, day = a2.Y - a1.Y;
      double uB = dby * dax - dbx * day;

      // ReSharper disable once CompareOfFloatsByEqualityOperator
      if (uB != 0.0) {
        double ua = (dbx * (a1.Y - b1.Y) - dby * (a1.X - b1.X)) / uB;
        return new PointDouble(a1.X - ua * -dax, a1.Y - ua * -day);
      }
      return null;

    }


    private static double Distance(PointDouble a, PointDouble b) {
      double dx = a.X - b.X;
      double dy = a.Y - b.Y;
      return dx * dx + dy * dy;
    }


    private static bool IsOnSegment(double xi, double yi, double xj, double yj, double xk, double yk) {
      return (xi <= xk || xj <= xk) && (xk <= xi || xk <= xj) &&
             (yi <= yk || yj <= yk) && (yk <= yi || yk <= yj);
    }


    private static int ComputeDirection(double xi, double yi, double xj, double yj, double xk, double yk) {
      double a = (xk - xi) * (yj - yi);
      double b = (xj - xi) * (yk - yi);
      return a < b ? -1 : a > b ? 1 : 0;
    }


    private static bool DoLineSegmentsIntersect(double x1, double y1, double x2, double y2, double x3, double y3,
      double x4, double y4) {
      int d1 = ComputeDirection(x3, y3, x4, y4, x1, y1);
      int d2 = ComputeDirection(x3, y3, x4, y4, x2, y2);
      int d3 = ComputeDirection(x1, y1, x2, y2, x3, y3);
      int d4 = ComputeDirection(x1, y1, x2, y2, x4, y4);
      return (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
              ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) ||
             (d1 == 0 && IsOnSegment(x3, y3, x4, y4, x1, y1)) ||
             (d2 == 0 && IsOnSegment(x3, y3, x4, y4, x2, y2)) ||
             (d3 == 0 && IsOnSegment(x1, y1, x2, y2, x3, y3)) ||
             (d4 == 0 && IsOnSegment(x1, y1, x2, y2, x4, y4));
    }


    private static int Parent(int index) {
      //return Math.Floor((index - 1) / 2);      
      return (index - 1) / 2;
    }


    private static int Child(int index) {
      return 2 * index + 1;
    }


    private static double Angle2(PointDouble a, PointDouble b, PointDouble c) {
      double a1 = Angle(a, b);
      double a2 = Angle(b, c);
      double a3 = a1 - a2;
      if (a3 < 0.0) {
        a3 += 360.0;
      }
      if (a3 > 360.0) {
        a3 -= 360.0;
      }
      //a3 %= 360.0; // NOT equivalent
      return a3;
    }


    private static SegmentPointAngle[] SortPoints(PointDouble position, List<SegmentDouble> segments) {
      int segCount = segments.Count;
      var points = new SegmentPointAngle[segCount * 2];
      for (var i = 0; i < segCount; ++i) {
        for (var j = 0; j < 2; ++j) {
          double a = Angle(segments[i][j], position);
          points[2 * i + j] = new SegmentPointAngle(i, j, a);
        }
      }

      // sort by angle
      //points.sort(function(a,b) {return a[2]-b[2];});
      Array.Sort(points, (a, b) => a.Angle.CompareTo(b.Angle));

      return points;
    }


    private static double Angle(PointDouble a, PointDouble b) {
      return Math.Atan2(b.Y - a.Y, b.X - a.X) * 180.0 / Math.PI;
    }


    private static bool Equal(PointDouble? aNull, PointDouble? bNull) {
      // fix
      if (!aNull.HasValue || !bNull.HasValue) {
        return false;
      }

      PointDouble a = aNull.Value;
      PointDouble b = bNull.Value;
      // end of fix

      return Math.Abs(a.X - b.X) < Epsilon && Math.Abs(a.Y - b.Y) < Epsilon;
    }


    private static bool LessThan(int index1, int index2, PointDouble position, List<SegmentDouble> segments, PointDouble destination) {
      PointDouble? inter1Null = IntersectLines(segments[index1][0], segments[index1][1], position, destination);
      PointDouble? inter2Null = IntersectLines(segments[index2][0], segments[index2][1], position, destination);

      // fix
      if (!inter1Null.HasValue || !inter2Null.HasValue) {
        return false;
      }
      // end of fix

      PointDouble inter1 = inter1Null.Value;
      PointDouble inter2 = inter2Null.Value;
      if (!Equal(inter1, inter2)) {
        double d1 = Distance(inter1, position);
        double d2 = Distance(inter2, position);
        return d1 < d2;
      }

      var end1 = 0;
      if (Equal(inter1, segments[index1][0])) {
        end1 = 1;
      }

      var end2 = 0;
      if (Equal(inter2, segments[index2][0])) {
        end2 = 1;
      }

      double a1 = Angle2(segments[index1][end1], inter1, position);
      double a2 = Angle2(segments[index2][end2], inter2, position);
      if (a1 < 180.0) {
        if (a2 > 180.0) {
          return true;
        }
        return a2 < a1;
      }
      return a1 < a2;
    }


    public static List<SegmentDouble> ConvertToSegments(List<List<PointDouble>> polygons) {
      if (polygons == null) {
        throw new ArgumentNullException(nameof(polygons));
      }

      var segments = new List<SegmentDouble>();
      foreach (List<PointDouble> polygon in polygons) {
        for (var j = 0; j < polygon.Count; ++j) {
          int k = j + 1;
          if (k == polygon.Count) {
            k = 0;
          }

          // segments.push([[polygons[i][j][0], polygons[i][j][1]], [polygons[i][k][0], polygons[i][k][1]]]);
          segments.Add(new SegmentDouble(polygon[j], polygon[k]));
        }
      }
      return segments;
    }


    public static bool InPolygon(PointDouble position, List<PointDouble> polygon) {
      if (polygon == null) {
        throw new ArgumentNullException(nameof(polygon));
      }

      double val = polygon[0].X;
      foreach (PointDouble point in polygon) {
        val = Math.Min(point.X, val);
        val = Math.Min(point.Y, val);
      }

      var edge = new PointDouble(val - 1, val - 1);
      var parity = 0;
      for (var i = 0; i < polygon.Count; ++i) {
        int j = i + 1;
        if (j == polygon.Count) {
          j = 0;
        }
        if (DoLineSegmentsIntersect(edge.X, edge.Y, position.X, position.Y, polygon[i].X, polygon[i].Y, polygon[j].X,
          polygon[j].Y)) {
          // intersect should have a value if we got here
          PointDouble? intersect = IntersectLines(edge, position, polygon[i], polygon[j]);
          if (Equal(position, intersect)) {
            return true;
          }
          if (Equal(intersect, polygon[i])) {
            if (Angle2(position, edge, polygon[j]) < 180.0) {
              ++parity;
            }
          }
          else if (Equal(intersect, polygon[j])) {
            if (Angle2(position, edge, polygon[i]) < 180.0) {
              ++parity;
            }
          }
          else {
            ++parity;
          }
        }
      }
      return (parity % 2) != 0;
    }


    private static void Splice<T>(List<T> source, int index, int count) {
      source.RemoveRange(index, count);
    }


    public static List<SegmentDouble> BreakIntersections(List<SegmentDouble> segments) {
      var output = new List<SegmentDouble>();
      for (var i = 0; i < segments.Count; ++i) {
        var intersections = new List<PointDouble>();
        for (var j = 0; j < segments.Count; ++j) {
          if (i == j) {
            continue;
          }
          if (DoLineSegmentsIntersect(segments[i][0].X, segments[i][0].Y, segments[i][1].X, segments[i][1].Y,
            segments[j][0].X, segments[j][0].Y, segments[j][1].X, segments[j][1].Y)) {
            PointDouble? intersect = IntersectLines(segments[i][0], segments[i][1], segments[j][0], segments[j][1]);
            if (intersect == null) {
              continue;
            }
            if (Equal(intersect.Value, segments[i][0]) || Equal(intersect.Value, segments[i][1])) {
              continue;
            }
            intersections.Add(intersect.Value);
          }
        }

        //PointDouble start = new PointDouble(segments[i][0].X, segments[i][0].Y);
        PointDouble start = segments[i][0];
        while (intersections.Count > 0) {
          var endIndex = 0;
          double endDis = Distance(start, intersections[0]);
          for (var j = 1; j < intersections.Count; ++j) {
            double dis = Distance(start, intersections[j]);
            if (dis < endDis) {
              endDis = dis;
              endIndex = j;
            }
          }
          output.Add(new SegmentDouble(start, intersections[endIndex]));
          start = intersections[endIndex];
          //intersections.splice(endIndex, 1);
          Splice(intersections, endIndex, 1);
        }
        // output.push([start, [segments[i][1][0], segments[i][1][1]]]);
        output.Add(new SegmentDouble(start, segments[i][1]));
      }
      return output;
    }


    private static int Pop(List<int> heap) {
      if (heap.Count > 0) {
        int index = heap.Count - 1;
        int val = heap[index];
        heap.RemoveAt(index);
        return val;
      }
      //throw new InvalidOperationException();
      return 0;
    }


    private static void Remove(int index, List<int> heap, PointDouble position, List<SegmentDouble> segments, PointDouble destination,
      int[] map) {
      map[heap[index]] = -1;
      if (index == heap.Count - 1) {
        Pop(heap);
        return;
      }
      heap[index] = Pop(heap);
      map[heap[index]] = index;
      int cur = index;
      int parent = Parent(cur);
      if (cur != 0 && LessThan(heap[cur], heap[parent], position, segments, destination)) {
        while (cur > 0) {
          parent = Parent(cur);
          if (!LessThan(heap[cur], heap[parent], position, segments, destination)) {
            break;
          }
          map[heap[parent]] = cur;
          map[heap[cur]] = parent;
          int temp = heap[cur];
          heap[cur] = heap[parent];
          heap[parent] = temp;
          cur = parent;
        }
      }
      else {
        while (true) {
          int left = Child(cur);
          int right = left + 1;
          if (left < heap.Count && LessThan(heap[left], heap[cur], position, segments, destination) &&
              (right == heap.Count || LessThan(heap[left], heap[right], position, segments, destination))) {
            map[heap[left]] = cur;
            map[heap[cur]] = left;
            int temp = heap[left];
            heap[left] = heap[cur];
            heap[cur] = temp;
            cur = left;
          }
          else if (right < heap.Count && LessThan(heap[right], heap[cur], position, segments, destination)) {
            map[heap[right]] = cur;
            map[heap[cur]] = right;
            int temp = heap[right];
            heap[right] = heap[cur];
            heap[cur] = temp;
            cur = right;
          }
          else {
            break;
          }
        }
      }
    }


    private static void Insert(int index, List<int> heap, PointDouble position, List<SegmentDouble> segments, PointDouble destination,
      int[] map) {
      PointDouble? intersect = IntersectLines(segments[index][0], segments[index][1], position, destination);
      if (intersect == null) {
        return;
      }
      int cur = heap.Count;
      heap.Add(index);
      map[index] = cur;
      while (cur > 0) {
        int parent = Parent(cur);
        if (!LessThan(heap[cur], heap[parent], position, segments, destination)) {
          break;
        }
        map[heap[parent]] = cur;
        map[heap[cur]] = parent;
        int temp = heap[cur];
        heap[cur] = heap[parent];
        heap[parent] = temp;
        cur = parent;
      }
    }

    private class SegmentPointAngle
    {
      public int SegmentIndex { get; }
      public int PointIndex { get; }
      public double Angle { get; }

      public SegmentPointAngle(int seg, int point, double a) {
        SegmentIndex = seg;
        PointIndex = point;
        Angle = a;
      }
    }
  }
}
