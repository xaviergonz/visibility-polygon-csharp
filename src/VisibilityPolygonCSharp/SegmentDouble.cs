namespace VisibilityPolygonCSharp
{
  /// <summary>
  ///   Immutable class that holds a segment (two double points).
  /// </summary>
  public class SegmentDouble
  {
    public SegmentDouble(PointDouble p1, PointDouble p2)
    {
      P1 = p1;
      P2 = p2;
    }

    public PointDouble P1 { get; }
    public PointDouble P2 { get; }

    internal PointDouble this[int index]
    {
      get
      {
        if (index == 0)
        {
          return P1;
        }
        return P2;
      }
    }
  }
}