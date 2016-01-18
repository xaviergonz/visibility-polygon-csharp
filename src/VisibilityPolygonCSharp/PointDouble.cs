namespace VisibilityPolygonCSharp
{
  /// <summary>
  ///   Immutable class that holds a point with double coordinates.
  /// </summary>
  public struct PointDouble
  {
    public double X { get; }
    public double Y { get; }

    public PointDouble(double x, double y)
    {
      X = x;
      Y = y;
    }

    public override string ToString()
    {
      return $"({X}, {Y})";
    }
  }
}