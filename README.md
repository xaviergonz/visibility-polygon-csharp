# VisibilityPolygonCSharp v1.9

This library constructs a [visibility polygon](https://en.wikipedia.org/wiki/Visibility_polygon) for a set of line segments. It is a C# port of the excellent [visibility-polygon-js](https://github.com/byronknoll/visibility-polygon-js) library.

**Latest NuGet package:** [VisibilityPolygonCSharp](https://www.nuget.org/packages/VisibilityPolygonCSharp/)

This code is released under the MIT License. See the [LICENSE](LICENSE) file for details.

- **Original library by:** [Byron Knoll](https://github.com/byronknoll/visibility-polygon-js)
- **C# Port by:** Javier González Garcés ([GitHub repository](https://github.com/xaviergonz/visibility-polygon-csharp))
- **JavaScript demo:** [Visibility Demo](http://www.byronknoll.com/visibility.html)

The time complexity of this implementation is **O(N log N)**, where *N* is the total number of line segments. This is the optimal time complexity for this problem.

> **Note:** The following documentation is based on the JavaScript port and will be updated for the C# port soon.

## Functions

### 1. `VisibilityPolygon.compute(position, segments)`

Computes a visibility polygon. **Time complexity:** O(N log N), where *N* is the number of line segments.

**Arguments:**
- **position:** The observer’s location. If the observer is not completely surrounded by line segments, an outer bounding box will be automatically created so that the visibility polygon does not extend to infinity.
- **segments:** A list of line segments. Each segment should be an array of two points, with each point being an array of two coordinates. Line segments must not intersect each other. Overlapping vertices are allowed, but a vertex touching the middle of a segment is not. Use the `breakIntersections` function to fix intersecting segments.

**Returns:** The visibility polygon (in clockwise vertex order).

### 2. `VisibilityPolygon.computeViewport(position, segments, viewportMinCorner, viewportMaxCorner)`

Computes a visibility polygon within the given viewport. This can be faster than `compute` if many segments lie outside the viewport.

**Arguments:**
- **position:** The observer’s location (must be within the viewport).
- **segments:** A list of line segments. Line segments must not intersect each other; however, segments may intersect the viewport.
- **viewportMinCorner:** An array specifying the minimum X and Y coordinates of the viewport.
- **viewportMaxCorner:** An array specifying the maximum X and Y coordinates of the viewport.

**Returns:** The visibility polygon within the viewport (in clockwise vertex order).

### 3. `VisibilityPolygon.inPolygon(position, polygon)`

Determines whether a point is within a polygon. **Time complexity:** O(N), where *N* is the number of points in the polygon.

**Arguments:**
- **position:** The point to check, as an array of two coordinates.
- **polygon:** The polygon defined as an array of points. The vertices may be in clockwise or counterclockwise order.

**Returns:** `true` if the point is within the polygon; otherwise, `false`.

### 4. `VisibilityPolygon.convertToSegments(polygons)`

Converts the given polygons into a list of line segments. **Time complexity:** O(N), where *N* is the number of polygons.

**Arguments:** A list of polygons. Each polygon is an array of points (each point is an array of two coordinates), with vertices in either clockwise or counterclockwise order.

**Returns:** A list of line segments.

### 5. `VisibilityPolygon.breakIntersections(segments)`

Breaks apart line segments so that none of them intersect. **Time complexity:** O(N²), where *N* is the number of line segments.

**Arguments:** A list of line segments. Each segment is an array of two points, and each point is an array of two coordinates.

**Returns:** A list of non-intersecting line segments.

## Example Code

```cs
var polygons = [];

polygons.push([[-1, -1], [501, -1], [501, 501], [-1, 501]]);
polygons.push([[250, 100], [260, 140], [240, 140]]);

var segments = VisibilityPolygon.convertToSegments(polygons);
segments = VisibilityPolygon.breakIntersections(segments);

var position = [60, 60];

if (VisibilityPolygon.inPolygon(position, polygons[0])) {
  var visibility = VisibilityPolygon.compute(position, segments);
}

var viewportVisibility = VisibilityPolygon.computeViewport(position, segments, [50, 50], [450, 450]);
