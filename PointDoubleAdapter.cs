namespace VisibilityPolygonCSharp
{
  /// <summary>
  ///   A sample point adapter for the sample point 
  /// </summary>
  public class PointDoubleAdapter : PointAdapter<PointDouble>
  {
    public override double GetX(PointDouble point)
    {
      return point.X;
    }

    public override double GetY(PointDouble point)
    {
      return point.Y;
    }

    public override PointDouble Create(double x, double y)
    {
      return new PointDouble(x, y);
    }
  }
}