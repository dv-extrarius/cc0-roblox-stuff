--[[
Copyright Extrarius 2024.
This work is marked with CC0 1.0 Universal.
To view a copy of this license, visit http://creativecommons.org/publicdomain/zero/1.0
--
This is a very basic implementation of delaunay triangulation using the Bowyer–Watson algorithm.
This code works in 2d using the X and Z coordinates of a Vector3. Y is essentially ignored.
--]]
--!strict
--!optimize 2

local Triangulation = {}

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
Triangulation.Triangulate = Triangulate

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
Triangulation.TrianglesToGraph = TrianglesToGraph

return table.freeze(Triangulation)