--[[
Copyright Extrarius 2024.
This work is marked with CC0 1.0 Universal.
To view a copy of this license, visit http://creativecommons.org/publicdomain/zero/1.0
--
This is a very basic implementation of delaunay triangulation using the Bowyer–Watson algorithm,
as well as a basic implementation of prim's algorithm for minimum spanning tree.
It also contains random utility functions helpful to turn a point cloud into a "star map"
--]]
--!strict
--!optimize 2

local PointGraph = {}


export type Triangle = {Points: {Vector3}, CircleCenter: Vector3, CircleRadius: number}
export type PointGraph = {[Vector3]: {[Vector3]: true}}


--Compute the circumcircle of a triangle via intersecting two perpendicular bisectors
local function ComputeCircumCircle(triangle: Triangle)
    local a = triangle.Points[1]
    local b = triangle.Points[2]
    local c = triangle.Points[3]
    local bx = b.X
    local bz = b.Z
    local q1 = 0.5*(a + b)
    local d1 = Vector3.new(a.Z - bz, 0, bx - a.X)
    local q2 = 0.5*(b + c)
    local d2 = Vector3.new(bz - c.Z, 0, c.X - bx)

    --Q1x + T1*D1x = Q2x + T2*D2x
    --Q1z + T1*D1z = Q2z + T2*D2z

    local d2x = d2.X
    local d2z = d2.Z
    local t1 = (d2x*(q1.Z - q2.Z) + d2z*(q2.X - q1.X)) / (d1.X * d2z - d1.Z * d2x)

    local center = q1 + t1 * d1
    local radius = (center - a).Magnitude

    triangle.CircleCenter = center
    triangle.CircleRadius = radius
end


--Perform delaunay triangulation using the Bowyer–Watson algorithm
local function Triangulate(points: {Vector3}): {Triangle}
    --Compute center and radius of points, needed for initial triangle
    local center = Vector3.zero
    for _, point in points do
        center += point
    end
    center /= #points

    local maxDistance = 1 --smallest maxDistance we allow
    for _, point in points do
        maxDistance = math.max(maxDistance, (point - center).Magnitude)
    end

    --Initial triangle that contains all points (is 5*maxDistance^2 big enough?)
    local sizeMultiplier = 5 * maxDistance * maxDistance
    local outerTriangle: Triangle = {
        Points = {
            sizeMultiplier * Vector3.new(math.sqrt(3), 0, 1) + center,
            sizeMultiplier * Vector3.new(-math.sqrt(3), 0, 1) + center,
            sizeMultiplier * Vector3.new(0, 0, -2) + center,
        },
        CircleCenter = Vector3.zero,
        CircleRadius = 0
    }
    ComputeCircumCircle(outerTriangle)

    ---
    local triangles: {[Triangle]: true} = {[outerTriangle] = true}
    local badTriangles: {[Triangle]: true} = {}
    local polygons: {{Vector3}} = {}

    --Map edges to triangles that share that edge
    local pointConnectivity: {[Vector3]: {[Vector3]: {[Triangle]: true}}} = {}

    pointConnectivity[outerTriangle.Points[1]] = {
        [outerTriangle.Points[2]] = {[outerTriangle] = true},
        [outerTriangle.Points[3]] = {[outerTriangle] = true},
    }
    pointConnectivity[outerTriangle.Points[2]] = {
        [outerTriangle.Points[3]] = {[outerTriangle] = true},
        [outerTriangle.Points[1]] = {[outerTriangle] = true},
    }
    pointConnectivity[outerTriangle.Points[3]] = {
        [outerTriangle.Points[1]] = {[outerTriangle] = true},
        [outerTriangle.Points[2]] = {[outerTriangle] = true},
    }

    for _, point in points do
        --Find triangles that contain the point
        for tri in triangles do
            if (point - tri.CircleCenter).Magnitude <= tri.CircleRadius then
                badTriangles[tri] = true
            end
        end
        --See if any edges from badTriangls should be reused
        for tri in badTriangles do
            for _, edge in {{1, 2}, {1, 3}, {2, 3}} do
                local useEdge = true
                local a = tri.Points[edge[1]]
                local b = tri.Points[edge[2]]
                --Don't need to check swapped since pointConnectivity contains all orders
                for tri2 in pointConnectivity[a][b] do
                    if tri == tri2 then
                        continue
                    elseif badTriangles[tri2] == true then
                        useEdge = false
                        break
                    end
                end
                if useEdge then
                    table.insert(polygons, {a, b})
                end
            end
        end
        --Remove bad triangles
        for tri in badTriangles do
            triangles[tri] = nil
            for _, edge in {{1, 2}, {1, 3}, {2, 3}} do
                local a = tri.Points[edge[1]]
                local b = tri.Points[edge[2]]
                pointConnectivity[a][b][tri] = nil
                pointConnectivity[b][a][tri] = nil
            end
        end
        --Add new triangles with point
        for _, edge in polygons do
            local tri: Triangle = {
                Points = {
                    edge[1],
                    edge[2],
                    point
                },
                CircleCenter = Vector3.zero,
                CircleRadius = 0
            }
            ComputeCircumCircle(tri)
            --Insert new triangle into connectivity
            for _, edge in {{1, 2}, {1, 3}, {2, 3}} do
                local a = tri.Points[edge[1]]
                local b = tri.Points[edge[2]]
                local connectivity1 = pointConnectivity[a]
                if not connectivity1 then
                    connectivity1 = {}
                    pointConnectivity[a] = connectivity1
                end
                local connectivity2 = connectivity1[b]
                if not connectivity2 then
                    connectivity2 = {}
                    connectivity1[b] = connectivity2
                end
                connectivity2[tri] = true
                --
                connectivity1 = pointConnectivity[b]
                if not connectivity1 then
                    connectivity1 = {}
                    pointConnectivity[b] = connectivity1
                end
                connectivity2 = connectivity1[a]
                if not connectivity2 then
                    connectivity2 = {}
                    connectivity1[a] = connectivity2
                end
                connectivity2[tri] = true
            end
            --Insert new triangle into general list
            triangles[tri] = true
        end
        ----
        table.clear(badTriangles)
        table.clear(polygons)
    end
    --Remove triangles with a point that matches the outer triangle
    for _, vert in outerTriangle.Points do
        for _, triSet in pointConnectivity[vert] do
            for tri in triSet do
                triangles[tri] = nil
            end
        end
    end
    --Convert the triangle set into an array
    local result: {Triangle} = {}
    for tri in triangles do
        table.insert(result, tri)
    end
    return result
end
PointGraph.Triangulate = Triangulate


local function TrianglesToGraph(triangles: {Triangle}): PointGraph
    local graph: PointGraph = {}
    for _, tri in triangles do
        local points = tri.Points
        local point1 = points[1]
        local point2 = points[2]
        local point3 = points[3]
        --Edges 1-2 and 1-3
        local entry = graph[point1]
        if not entry then
            entry = {}
            graph[point1] = entry
        end
        entry[point2] = true
        entry[point3] = true
        --Edges 2-1 and 2-3
        entry = graph[point2]
        if not entry then
            entry = {}
            graph[point2] = entry
        end
        entry[point1] = true
        entry[point3] = true
        --Edges 3-1 and 3-2
        entry = graph[point3]
        if not entry then
            entry = {}
            graph[point3] = entry
        end
        entry[point1] = true
        entry[point2] = true
    end
    return graph
end
PointGraph.TrianglesToGraph = TrianglesToGraph


local function CloneGraph(graph: PointGraph): PointGraph
    local result: PointGraph = table.clone(graph)
    for vert1, vert2set in result do
        result[vert1] = table.clone(vert2set)
    end
    return result
end
PointGraph.CloneGraph = CloneGraph


local function IsGraphEmpty(graph: PointGraph): boolean
    for vert1, vert2set in graph do
        if next(vert2set) then
            return false
        end
    end
    return true
end
PointGraph.IsGraphEmpty = IsGraphEmpty


--Compute a minimum spanning tree of a graph using prim's algorithm
local function MinimumSpanningTree(graph: PointGraph): PointGraph
    local result: PointGraph = {}
    local unexploredEdges: PointGraph = CloneGraph(graph)

    --Make sure all disconnected islands are fully explored
    while not IsGraphEmpty(unexploredEdges) do
        --Find a random initial starting point
        do
            local points: {Vector3} = {}
            for vert1 in unexploredEdges do
                table.insert(points, vert1)
            end
            result[points[math.random(1, #points)]] = {}
        end
        --Add edges until all points connected to result points are added
        repeat
            local bestVertA = Vector3.zero
            local bestVertB = Vector3.zero
            local bestCost = math.huge
            local bestTieCount = 1

            --Iterate over all points in the result
            for vert1 in result do
                --Iterate over every point connected to the point that we haven't added yet
                for vert2 in unexploredEdges[vert1] do
                    local cost = (vert1 - vert2).Magnitude
                    if cost < bestCost then
                        bestVertA = vert1
                        bestVertB = vert2
                        bestCost = cost
                        bestTieCount = 1
                    elseif cost == bestCost then
                        --break ties randomly
                        bestTieCount += 1
                        if math.random(1, bestTieCount) == 1 then
                            bestVertA = vert1
                            bestVertB = vert2
                        end
                    end
                end
            end
            --If there is a good edge to insert
            if bestCost < math.huge then
                --Insert A->B link
                local entry = result[bestVertA]
                if not entry then
                    entry = {}
                    result[bestVertA] = entry
                end
                entry[bestVertB] = true
                --Insert B->A link
                entry = result[bestVertB]
                if not entry then
                    entry = {}
                    result[bestVertB] = entry
                end
                entry[bestVertA] = true
                --Every connection between bestVertB and points in result now leads to an explored area
                for vert1 in result do
                    unexploredEdges[vert1][bestVertB] = nil
                    unexploredEdges[bestVertB][vert1] = nil
                end
            end
            --Otherwise, exit the loop
        until bestCost == math.huge
    end
    return result
end
PointGraph.MinimumSpanningTree = MinimumSpanningTree


--Find all edges in the graph that intersect the given line. Works in 2D using X/Z
local function FindEdgesByLine(graph: PointGraph, lineStart: Vector3, lineEnd: Vector3): PointGraph
    local result: PointGraph = {}
    local lineDelta = lineEnd - lineStart
    local lineStartX = lineStart.X
    local lineStartZ = lineStart.Z
    local lineDeltaX = lineDelta.X
    local lineDeltaZ = lineDelta.Z

    for vert1, vert1Edges in graph do
        for vert2 in vert1Edges do
            --Only do the calculation for one direction of the edge
            if (vert2.X > vert1.X) or ((vert2.X == vert1.X) and (vert2.Z > vert1.Z)) then
                continue
            end
            --Test whether the line and edge intersect
            local vertDelta = vert2 - vert1
            local vertDeltaX = vertDelta.X
            local vertDeltaZ = vertDelta.Z
            local denom = lineDeltaZ*vertDeltaX - lineDeltaX*vertDeltaZ
            local mulX = (vert1.Z - lineStartZ)
            local mulZ = (lineStartX - vert1.X)

            local t1 = (mulX*vertDeltaX + mulZ*vertDeltaZ) / denom
            local t2 = (mulX*lineDeltaX + mulZ*lineDeltaZ) / denom

            if (t1 >= 0) and (t1 <= 1) and (t2 >= 0) and (t2 <= 1) then
                --Line and edge cross, remove edge
                local entry = result[vert1]
                if not entry then
                    entry = {}
                    result[vert1] = entry
                end
                entry[vert2] = true

                entry = result[vert2]
                if not entry then
                    entry = {}
                    result[vert2] = entry
                end
                entry[vert1] = true
            end
        end
    end
    return result
end
PointGraph.FindEdgesByLine = FindEdgesByLine


--Remove all edges in the graph that intersect the given line. Works in 2D using X/Z
local function CutEdgesByLine(graph: PointGraph, lineStart: Vector3, lineEnd: Vector3)
    local result = FindEdgesByLine(graph, lineStart, lineEnd)

    for vert1, vert2set in result do
        for vert2 in vert2set do
            graph[vert1][vert2] = nil
        end
    end
end
PointGraph.CutEdgesByLine = CutEdgesByLine


--Get points nearby according to straigt-line distance
local function GetNearbyUsingDistance(graph: PointGraph, startPoint: Vector3, maxDistance: number): {[Vector3]: number}
    local distances: {[Vector3]: number} = {}
    for vert1 in graph do
        local distance = (vert1 - startPoint).Magnitude
        if (distance <= maxDistance) then
            distances[vert1] = distance
        end
    end
    --don't return the point itself
    distances[startPoint] = nil

    return distances
end
PointGraph.GetNearbyUsingDistance = GetNearbyUsingDistance


--Get points nearby according to minimum path length
local function GetNearbyUsingPathLength(graph: PointGraph, startPoint: Vector3, maxPathLength: number): {[Vector3]: number}
    local pathLengths: {[Vector3]: number} = {}
    local currentWave: {[Vector3]: true} = {[startPoint] = true}
    local nextWave: {[Vector3]: true} = {}

    pathLengths[startPoint] = 0
    while next(currentWave) do
        for vert1 in currentWave do
            local baseLength = pathLengths[vert1]
            for vert2 in graph[vert1] do
                local pathLength = baseLength + (vert1 - vert2).Magnitude
                if (pathLength <= maxPathLength) and (pathLength < (pathLengths[vert2] or math.huge)) then
                    pathLengths[vert2] = pathLength
                    nextWave[vert2] = true
                end
            end
        end
        currentWave, nextWave = nextWave, currentWave
        table.clear(nextWave)
    end
    pathLengths[startPoint] = nil

    return pathLengths
end
PointGraph.GetNearbyUsingPathLength = GetNearbyUsingPathLength


return table.freeze(PointGraph)