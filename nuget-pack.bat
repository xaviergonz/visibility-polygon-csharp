@echo off
echo Creating nuget package...
nuget pack VisibilityPolygonCSharp.csproj -Prop Configuration=Release
echo Done!