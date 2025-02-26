# VisibilityPolygonCSharp v1.9
This library can be used to construct a visibility polygon for a set of line segments. 
Based on the excellent https://github.com/byronknoll/visibility-polygon-js

Latest NuGet package: https://www.nuget.org/packages/VisibilityPolygonCSharp/

This code is released into under the MIT License. Check the LICENSE file for details.
Original library made by Byron Knoll.
C# Port made by Javier González Garcés.

https://github.com/xaviergonz/visibility-polygon-csharp
JavaScript original version demo: http://www.byronknoll.com/visibility.html

This library can be used to construct a visibility polygon for a set of line segments.

The time complexity of this implementation is O(N log N) (where N is the total number of line segments). This is the optimal time complexity for this problem.

Note, the following documentation comes from the JavaScript port documentation.
It will be updated for the C# port soon-ish.

The following functions should be useful:

1) `VisibilityPolygon.compute(position, segments)`
   
  Computes a visibility polygon. O(N log N) time complexity (where N is the number of line segments).
  
  Arguments:
    - position - The location of the observer. If the observer is not completely surrounded by line segments, an outer bounding-box will be automatically created (so that the visibility polygon does not extend to infinity).
    - segments - A list of line segments. Each line segment should be a list of two points. Each point should be a list of two coordinates. Line segments can not intersect each other. Overlapping vertices are OK, but it is not OK if a vertex is touching the middle of a line segment. Use the "breakIntersections" function to fix intersecting line segments.
  
  Returns: The visibility polygon (in clockwise vertex order).

3) `VisibilityPolygon.computeViewport(position, segments, viewportMinCorner, viewportMaxCorner)`

  Computes a visibility polygon within the given viewport. This can be faster than the "compute" function if there are many segments outside of the viewport.
  
  Arguments:
    - position - The location of the observer. Must be within the viewport.
    - segments - A list of line segments. Line segments can not intersect each other. It is OK if line segments intersect the viewport.
    - viewportMinCorner - The minimum X and Y coordinates of the viewport.
    - viewportMaxCorner - The maximum X and Y coordinates of the viewport.
  
  Returns: The visibility polygon within the viewport (in clockwise vertex order).

5) `VisibilityPolygon.inPolygon(position, polygon)`

  Calculates whether a point is within a polygon. O(N) time complexity (where N is the number of points in the polygon).
  
  Arguments:
    - position - The point to check: a list of two coordinates.
    - polygon - The polygon to check: a list of points. The polygon can be specified in either clockwise or counterclockwise vertex order.
  
  Returns: True if "position" is within the polygon.

7) `VisibilityPolygon.convertToSegments(polygons)`

  Converts the given polygons to list of line segments. O(N) time complexity (where N is the number of polygons).
  
  Arguments: a list of polygons (in either clockwise or counterclockwise vertex order). Each polygon should be a list of points. Each point should be a list of two coordinates.
  
  Returns: a list of line segments.

9) `VisibilityPolygon.breakIntersections(segments)`

  Breaks apart line segments so that none of them intersect. O(N^2) time complexity (where N is the number of line segments).

  Arguments: a list of line segments. Each line segment should be a list of two points. Each point should be a list of two coordinates.
  
  Returns: a list of line segments.

Example code:
```cs
var polygons = [];

polygons.push([[-1,-1],[501,-1],[501,501],[-1,501]]);

polygons.push([[250,100],[260,140],[240,140]]);

var segments = VisibilityPolygon.convertToSegments(polygons);

segments = VisibilityPolygon.breakIntersections(segments);

var position = [60, 60];

if (VisibilityPolygon.inPolygon(position, polygons[0])) {

  var visibility = VisibilityPolygon.compute(position, segments);

}

var viewportVisibility = VisibilityPolygon.computeViewport(position, segments, [50, 50], [450, 450]);
```
