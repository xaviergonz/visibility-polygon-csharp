namespace VisibilityPolygonCSharp
{
  /// <summary>
  ///   A sample basic struct that holds a point with double coordinates.
  /// </summary>
  public struct PointDouble 
  {
    public double X { get; set; }
    public double Y { get; set; }

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