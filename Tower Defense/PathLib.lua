--[[
Copyright Extrarius 2023.
This work is marked with CC0 1.0 Universal.
To view a copy of this license, visit http://creativecommons.org/publicdomain/zero/1.0
--
This is an implementation of an essentially 2d pathfinding system. It ignores Y coordinates, and
uses a grid of nodes that are manually marked blocked or unblocked by the library's user.
]]
--!strict
--!optimize 2

local PathLib = {}

--Constants used to index the PathNode type
local IDX_NEIGHBORS = 1
local IDX_POSITION = 2
local IDX_INDEX = 3
local IDX_RANDOMWEIGHT = 4
local IDX_TOTALCOST = 5
local IDX_BLOCKED = 6
local IDX_NEXTNODE = 7

type PathNode = {any} --Unfortuantely heterogeneous arrays not yet supported in type system
--{Neighbors: {PathNode}, Position: Vector3, Index: number, RandomWeight: number, TotalCost: number, Blocked: boolean, NextNode: PathNode?}
export type PathMesh = {Nodes: {[number]: PathNode}, IsFinalized: boolean, Generator: Random}
export type PathList = {Vector3}

--Path nodes are spaced every other stud
local GRID_COORD_SPACING = 2
--Path nodes are on odd-numbered studs
local GRID_COORD_OFFSET = 1

--[[
Before I knew Vector3 were value types, I made my own position-to-key system. After discovering
they are value types, a benchmark showed my system is about 5% faster for my cast, so here it is.
Since places won't be huge and nodes are limited to integer coordinates, we can pack the 3 coordinates
using some bits for each one. Since there are effectively 53 mantissa bits, I chose floor(53/3) bits
per coordinate, resulting in a multiplier of 2^17 = 131072. I keep room for 3 coordinates just in case
- for now, we only use X and Z. Since we want max negative coordinate to map to 0, we add half the
multiplier. This allows maps to span the region from -65536 to +65535
]]
local GRID_INDEX_MUL = 131072

--Actual module-local variables
local EnableDebugMessages: boolean = false


--Custom debug functions to avoid having to repeat `if EnableDebugMessages then` everywhere
local function CustomWarn(...: any)
    if EnableDebugMessages then
        warn(...)
    end
end
--Variadic error that also checks EnableDebugMessages
local function CustomError(level: number, ...: any)
    if EnableDebugMessages then
        local args = {...}
        for i, arg in args do
            args[i] = tostring(arg)
        end
        error(table.concat(args, ""), level + 1)
    end
end


--Convert a coordinate to a node index
local function CoordToIndex(x: number, z: number): number
    return (x + (GRID_INDEX_MUL/2)) + ((z + (GRID_INDEX_MUL/2)) * GRID_INDEX_MUL)
end


--Convert an adjustment to a coordinate to an adjustment to an index
local function DeltaCoordToIndex(x: number, z: number): number
    return (x) + (z * GRID_INDEX_MUL)
end

--Function to snap coordinates to pathing grid by rounding
local function ToPathGridRoundSingle(coord: number): number
    return math.round((coord - GRID_COORD_OFFSET) / GRID_COORD_SPACING) * GRID_COORD_SPACING + GRID_COORD_OFFSET
end
local function ToPathGridRound(coord: Vector3): Vector3
    return Vector3.new(ToPathGridRoundSingle(coord.X), 0, ToPathGridRoundSingle(coord.Z))
end
PathLib.ToPathGridRound = ToPathGridRound


--Function to snap coordinates to pathing grid by flooring
local function ToPathGridFloorSingle(coord: number): number
    return math.floor((coord - GRID_COORD_OFFSET) / GRID_COORD_SPACING) * GRID_COORD_SPACING + GRID_COORD_OFFSET
end
local function ToPathGridFloor(coord: Vector3): Vector3
    return Vector3.new(ToPathGridFloorSingle(coord.X), 0, ToPathGridFloorSingle(coord.Z))
end
PathLib.ToPathGridFloor = ToPathGridFloor


--Function to snap coordinates to pathing grid by ceiling
local function ToPathGridCeilSingle(coord: number): number
    return math.ceil((coord - GRID_COORD_OFFSET) / GRID_COORD_SPACING) * GRID_COORD_SPACING + GRID_COORD_OFFSET
end
local function ToPathGridCeil(coord: Vector3): Vector3
    return Vector3.new(ToPathGridCeilSingle(coord.X), 0, ToPathGridCeilSingle(coord.Z))
end
PathLib.ToPathGridCeil = ToPathGridCeil


--Function to fit an area to the grid
local function AlignAreaToGrid(min: Vector3, max: Vector3, includePartial: boolean?): (Vector3, Vector3)
    local minX = math.min(min.X, max.X)
    local minZ = math.min(min.Z, max.Z)
    local maxX = math.max(min.X, max.X)
    local maxZ = math.max(min.Z, max.Z)
    if not includePartial then
        return Vector3.new(ToPathGridCeilSingle(minX), 0, ToPathGridCeilSingle(minZ)), Vector3.new(ToPathGridFloorSingle(maxX), 0, ToPathGridFloorSingle(maxZ))
    else
        return Vector3.new(ToPathGridFloorSingle(minX), 0, ToPathGridFloorSingle(minZ)), Vector3.new(ToPathGridCeilSingle(maxX), 0, ToPathGridCeilSingle(maxZ))
    end
end
PathLib.AlignAreaToGrid = AlignAreaToGrid


--Initialize PathLib with the provided settings
function PathLib.Configure(config: {EnableDebugMessages: boolean?}): ()
    if config.EnableDebugMessages ~= nil then
        EnableDebugMessages = config.EnableDebugMessages and true or false --cast to boolean
    end
end


--Create an empty mesh
function PathLib.NewMesh(seed: number?): PathMesh
    return {
        Nodes = {},
        IsFinalized = false,
        Generator = if seed ~= nil then Random.new(seed) else Random.new() --Random.new(nil) fails
    }
end


--Clone a mesh. Finalizes the nodes if the original is finialized
function PathLib.Clone(mesh: PathMesh, skipFinalize: boolean?): PathMesh
    local table_clone = table.clone
    local table_create = table.create

    local newMesh: PathMesh = {Nodes = {}, IsFinalized = false, Generator = mesh.Generator:Clone()}
    local newNodes = newMesh.Nodes

    for index, current in mesh.Nodes do
        local node = table_clone(current)

        node[IDX_NEIGHBORS] = table_create(4)
        newNodes[index] = node
    end

    if mesh.IsFinalized and not skipFinalize then
        PathLib.FinalizeMesh(newMesh)
    end

    return newMesh
end


--Add nodes to a mesh to cover a specified region
function PathLib.AddNodes(mesh: PathMesh, min: Vector3, max: Vector3, includePartial: boolean?): ()
    local table_create = table.create

    local nodes = mesh.Nodes
    local gen = mesh.Generator

    min, max = AlignAreaToGrid(min, max, includePartial)

    for z = min.Z, max.Z, GRID_COORD_SPACING do
        local index = CoordToIndex(min.X, z)

        for x = min.X, max.X, GRID_COORD_SPACING do
            if nodes[index] == nil then
                nodes[index] = {
                    table_create(4),               --Neighbors
                    Vector3.new(x, 0, z),          --Position
                    index,                         --Index
                    (gen:NextNumber() / 100000.0), --RandomWeight
                    math.huge,                     --TotalCost
                    false,                         --Blocked
                    nil,                           --NextNode
                }
            end
            index += DeltaCoordToIndex(GRID_COORD_SPACING, 0)
        end
    end
end


--Do final processing on each node in the mesh
function PathLib.FinalizeMesh(mesh: PathMesh): ()
    local table_freeze = table.freeze

    local nodes = mesh.Nodes
    local neighbor: PathNode?

    --Connect each node to 4-way neighbors if they exist
    for index, current in nodes do
        local neighbors: {PathNode} = current[IDX_NEIGHBORS]

        neighbor = nodes[index + DeltaCoordToIndex(-GRID_COORD_SPACING, 0)]
        if neighbor ~= nil then
            table.insert(neighbors, neighbor)
        end

        neighbor = nodes[index + DeltaCoordToIndex( GRID_COORD_SPACING, 0)]
        if neighbor ~= nil then
            table.insert(neighbors, neighbor)
        end

        neighbor = nodes[index + DeltaCoordToIndex(0, -GRID_COORD_SPACING)]
        if neighbor ~= nil then
            table.insert(neighbors, neighbor)
        end

        neighbor = nodes[index + DeltaCoordToIndex(0,  GRID_COORD_SPACING)]
        if neighbor ~= nil then
            table.insert(neighbors, neighbor)
        end

        table_freeze(current[IDX_NEIGHBORS])
    end

    mesh.IsFinalized = true
    table_freeze(mesh.Nodes)
end


--Mark all nodes in the specified region as blocked
function PathLib.BlockNodes(mesh: PathMesh, min: Vector3, max: Vector3, includePartial: boolean?): ()
    local nodes = mesh.Nodes

    min, max = AlignAreaToGrid(min, max, includePartial)

    for z = min.Z, max.Z, GRID_COORD_SPACING do
        local index = CoordToIndex(min.X, z)

        for x = min.X, max.X, GRID_COORD_SPACING do
            local node = nodes[index]

            if node ~= nil then
                node[IDX_BLOCKED] = true
            end
            index += DeltaCoordToIndex(GRID_COORD_SPACING, 0)
        end
    end
end


--Mark all nodes in the specified region as unblocked
function PathLib.UnblockNodes(mesh: PathMesh, min: Vector3, max: Vector3, includePartial: boolean?): ()
    local nodes = mesh.Nodes

    min, max = AlignAreaToGrid(min, max, includePartial)

    for z = min.Z, max.Z, GRID_COORD_SPACING do
        local index = CoordToIndex(min.X, z)

        for x = min.X, max.X, GRID_COORD_SPACING do
            local node = nodes[index]

            if node ~= nil then
                node[IDX_BLOCKED] = false
            end
            index += DeltaCoordToIndex(GRID_COORD_SPACING, 0)
        end
    end
end


--Utility to print warnings for invalid start/finish locations
local function ValidateNode(label: string, location: Vector3, node: PathNode): boolean
    if node == nil then
        CustomWarn(label, " point of (", location, ") is outside path nodes")
        return false
    elseif node[IDX_BLOCKED] then
        CustomWarn(label, " node at (", location, ") is blocked!")
        return false
    end
    return true
end


--Update the costs so paths can be found from anywhere to finishNode
local function UpdateMeshCosts(mesh: PathMesh, finishNode: PathNode): boolean
    local table_clear = table.clear

    --Finding a path requires a finalized mesh
    if not mesh.IsFinalized then
        CustomError(1, "Attempted UpdateMeshCosts without finalizing the mesh.")
        return false
    end

    --Reset costs to maximum value
    for _, node in mesh.Nodes do
        node[IDX_TOTALCOST] = math.huge
        node[IDX_NEXTNODE] = nil
    end

    --Do a full expansion of nodes in waves until all unblocked nodes have been assigned the minimum cost to get there
    local frontWave: {[number]: PathNode} = {}
    local nextWave: {[number]: PathNode} = {}

    frontWave[finishNode[IDX_INDEX]] = finishNode
    finishNode[IDX_TOTALCOST] = 0

    while next(frontWave) ~= nil do
        for index, current in frontWave do
            local TotalCost = current[IDX_TOTALCOST] + 1  + current[IDX_RANDOMWEIGHT]

            for _, neighbor in current[IDX_NEIGHBORS] do
                if (not neighbor[IDX_BLOCKED]) and (TotalCost < neighbor[IDX_TOTALCOST]) then
                    neighbor[IDX_TOTALCOST] = TotalCost
                    nextWave[neighbor[IDX_INDEX]] = neighbor
                end
            end
        end
        frontWave, nextWave = nextWave, frontWave
        table_clear(nextWave)
    end

    return true
end


--Update the costs so paths can be found from anywhere to finish
function PathLib.UpdateMeshCosts(mesh: PathMesh, finish: Vector3): boolean
    local finishNodePos = ToPathGridRound(finish)
    local finishNode = mesh.Nodes[CoordToIndex(finishNodePos.X, finishNodePos.Z)]

    if not ValidateNode("Finish", finish, finishNode) then
        return false
    end

    return UpdateMeshCosts(mesh, finishNode)
end


--Find the next node towards the goal from each node in the mesh
local function CalculateNextNodes(mesh: PathMesh, finishNode: PathNode): ()
    if finishNode[IDX_NEXTNODE] ~= nil then
        return
    end

    --For each node, calculate the best neighbor for moving towards the finish node
    for index, node in mesh.Nodes do
        if node[IDX_BLOCKED] or (node[IDX_TOTALCOST] == math.huge) then
            continue
        end
        local bestNeighbor: PathNode? = nil
        local bestCost = math.huge

        --If the node itself isn't blocked and was visited, it's reachable so has at least one good neighbor
        for _, neighbor in node[IDX_NEIGHBORS] do
            local neighborCost: number = neighbor[IDX_TOTALCOST]
            if neighborCost < bestCost then
                bestNeighbor = neighbor
                bestCost = neighborCost
            end
        end
        --
        if (bestNeighbor == nil) or  (bestNeighbor[IDX_TOTALCOST] == math.huge) then
            CustomError(1, "Failed to find a good neighbor for a reachable node!")
        else
            node[IDX_NEXTNODE] = bestNeighbor
        end
    end
    finishNode[IDX_NEXTNODE] = finishNode
end


--Find a path between the given nodes using the already-computed costs
local function FindPathUsingCosts(mesh: PathMesh, startNode: PathNode, finishNode: PathNode): PathList?
    --If the finish node wasn't the origin, the costs can't be used for this path
    if finishNode[IDX_TOTALCOST] ~= 0 then
        CustomWarn("Finish node at (", finishNode[IDX_POSITION], ") was not the finish used to update costs!")
        return nil
    end

    --If the start node wasn't visited, no path can exist
    if startNode[IDX_TOTALCOST] == math.huge then
        CustomWarn("FindPathUsingCosts with start node (", startNode[IDX_POSITION], ") that is unreachable")
        return nil
    end

    CalculateNextNodes(mesh, finishNode)

    --Follow the cheapest nodes from start finish
    --Note that since start has a valid TotalCost, it was visited so a path exists
    --That means this code doesn't need to check for blocked or unvisited neighbors because better ones will exist
    local path: {PathNode} = {startNode}
    local pathLen = 1
    while path[pathLen] ~= finishNode do
        local current = path[pathLen]
        local bestNeighbor: PathNode? = current[IDX_NEXTNODE]

        --If a valid neighbor isn't found, no reason to keep searching (should never happen)
        if (bestNeighbor == nil) then
            CustomError(1, "FindPathUsingCosts failed to find path between reachable nodes! Start (", startNode[IDX_POSITION], ") to Finish (", finishNode[IDX_POSITION], ")")
            break
        end
        --Record the neighbor
        pathLen += 1
        path[pathLen] = bestNeighbor
    end

    --If the path didn't connect to the finish node, a path doesn't exist (should never happen)
    if path[pathLen] ~= finishNode then
        CustomError(1, "FindPathUsingCosts failed to find path between reachable nodes! Start (", startNode[IDX_POSITION], ") to Finish (", finishNode[IDX_POSITION], ")")
        return nil
    end

    --Store the position of each node used in the correct (reversed, so start to finish) order
    local result: PathList = table.create(pathLen)

    for i, node in path do
        result[i] = node[IDX_POSITION]
    end

    return result
end


--Find a path between the given locations using the already-computed costs
function PathLib.FindPathUsingCosts(mesh: PathMesh, start: Vector3, finish: Vector3): PathList?
    --Look up the start and finish locations in the node grid
    local startNodePos = ToPathGridRound(start)
    local finishNodePos = ToPathGridRound(finish)

    local startNode = mesh.Nodes[CoordToIndex(startNodePos.X, startNodePos.Z)]
    local finishNode = mesh.Nodes[CoordToIndex(finishNodePos.X, finishNodePos.Z)]

    if not ValidateNode("Start", start, startNode) then
        return nil
    end

    if not ValidateNode("Finish", finish, finishNode) then
        return nil
    end

    if finishNode[IDX_TOTALCOST] ~= 0 then
        CustomWarn("Finish node at (", finish, ") was not the finish used to update costs!")
        return nil
    end

    return FindPathUsingCosts(mesh, startNode, finishNode)
end


--Find a path between the given locations using the path mesh. Updates costs
function PathLib.FindPath(mesh: PathMesh, start: Vector3, finish: Vector3): PathList?
    --Look up the start and finish locations in the node grid
    local startNodePos = ToPathGridRound(start)
    local finishNodePos = ToPathGridRound(finish)

    local startNode = mesh.Nodes[CoordToIndex(startNodePos.X, startNodePos.Z)]
    local finishNode = mesh.Nodes[CoordToIndex(finishNodePos.X, finishNodePos.Z)]

    if not ValidateNode("Start", start, startNode) then
        return nil
    end

    if not ValidateNode("Finish", finish, finishNode) then
        return nil
    end

    if not UpdateMeshCosts(mesh, finishNode) then
        return nil
    end

    return FindPathUsingCosts(mesh, startNode, finishNode)
end


--[[
    Calculate whether a line between two points includes only unblocked nodes.
    NOTE: The points need not be aligned to the pathing grid!
    Returns false iff the line crosses any grid coordinate with no node or a node marked blocked.

    Based on "A Fast Voxel Traversal Algorithm for Ray Tracing" by John Amanatides and Andrew Woo
    Extended to be a kind of "circle cast" against the path grid by tracing two lines.
--]]
local function IsLineTraversable(mesh: PathMesh, start: Vector3, finish: Vector3): boolean
    --Radius can't exceed (or equal?) GRID_COORD_SPACING/2 or nodes might be missed
    local radius = GRID_COORD_SPACING/2 - 0.00001
    local nodes = mesh.Nodes
    local node: PathNode

    local delta = finish - start
    local du = delta.Unit --cache for speed
    --How far to step along the X and Z coordinates to get the next grid coordinate
    local stepX = if delta.X > 0 then GRID_COORD_SPACING else -GRID_COORD_SPACING
    local stepZ = if delta.Z > 0 then GRID_COORD_SPACING else -GRID_COORD_SPACING
    --How to adjust the index to get to the cell in the corresponding direction
    local offsetX = DeltaCoordToIndex(stepX, 0)
    local offsetZ = DeltaCoordToIndex(0, stepZ)

    --Two points opposite a circle by the normal of the line
    local pAX = start.X + radius *  du.Z
    local pAZ = start.Z + radius * -du.X
    local pBX = start.X + radius * -du.Z
    local pBZ = start.Z + radius *  du.X
    --Those coordinates snapped to the grid
    local pAXGrid = ToPathGridRoundSingle(pAX)
    local pAZGrid = ToPathGridRoundSingle(pAZ)
    local pBXGrid = ToPathGridRoundSingle(pBX)
    local pBZGrid = ToPathGridRoundSingle(pBZ)

    local index = CoordToIndex(pAXGrid, pAZGrid)
    --If the line is horizontal or vertical, special processinng is required to avoid division by zero
    if delta.X == 0 then
        --For the loop count we need to know
        local pAEndZ = finish.Z + radius * -du.X
        local pAEndZGrid = ToPathGridRoundSingle(pAEndZ)

        if pAXGrid == pBXGrid then
            for zz = pAZGrid, pAEndZGrid, stepZ do
                node = nodes[index]
                if not node or node[IDX_BLOCKED] then
                    return false
                end
                index += offsetZ
            end
        else
            local indexDelta
            if pAXGrid < pBXGrid then
                indexDelta = DeltaCoordToIndex(GRID_COORD_SPACING, 0)
            else
                indexDelta = DeltaCoordToIndex(-GRID_COORD_SPACING, 0)
            end

            for zz = pAZGrid, pAEndZGrid, stepZ do
                node = nodes[index]
                if not node or node[IDX_BLOCKED] then
                    return false
                end
                node = nodes[index + indexDelta]
                if not node or node[IDX_BLOCKED] then
                    return false
                end
                index += offsetZ
            end
        end
        return true
    elseif delta.Z == 0 then
        local pAEndX = finish.X + radius *  du.Z
        local pAEndXGrid = ToPathGridRoundSingle(pAEndX)

        if pAZGrid ~= pBZGrid then
            for xx = pAXGrid, pAEndXGrid, stepX do
                node = nodes[index]
                if not node or node[IDX_BLOCKED] then
                    return false
                end
                index += offsetX
            end
        else
            local indexDelta
            if pAZGrid < pBZGrid then
                indexDelta = DeltaCoordToIndex(0, GRID_COORD_SPACING)
            else
                indexDelta = DeltaCoordToIndex(0, GRID_COORD_SPACING)
            end

            for xx = pAXGrid, pAEndXGrid, stepX do
                node = nodes[index]
                if not node or node[IDX_BLOCKED] then
                    return false
                end
                node = nodes[index + indexDelta]
                if not node or node[IDX_BLOCKED] then
                    return false
                end
                index += offsetX
            end
        end
        return true
    end
    --When to step to keep the slope correct
    local tMaxXA = (pAXGrid + stepX/2 - pAX) / delta.X
    local tMaxZA = (pAZGrid + stepZ/2 - pAZ) / delta.Z
    local tMaxXB = (pBXGrid + stepX/2 - pBX) / delta.X
    local tMaxZB = (pBZGrid + stepZ/2 - pBZ) / delta.Z

    --The change in T a step in each direction adavances.
    local tDeltaX = stepX / delta.X
    local tDeltaZ = stepZ / delta.Z

    --Step line A between X and Z grid crossings, checking each node along the path
    while (tMaxXA <= 1) or (tMaxZA <= 1) do
        node = nodes[index]
        if not node or node[IDX_BLOCKED] then
            return false
        end
        --Advance to the next nearest grid crossing, whether in X or Z direction
        if tMaxXA < tMaxZA then
            tMaxXA += tDeltaX
            index += offsetX
        else
            tMaxZA += tDeltaZ
            index += offsetZ
        end
    end
    --Check the final node that line A hits
    node = nodes[index]
    if not node or node[IDX_BLOCKED] then
        return false
    end

    --Step line B between X and Z grid crossings, checking each node along the path
    index = CoordToIndex(pBXGrid, pBZGrid)
    while (tMaxXB <= 1) or (tMaxZB <= 1) do
        node = nodes[index]
        if not node or node[IDX_BLOCKED] then
            return false
        end
        --Advance to the next nearest grid crossing, whether in X or Z direction
        if tMaxXB < tMaxZB then
            tMaxXB += tDeltaX
            index += offsetX
        else
            tMaxZB += tDeltaZ
            index += offsetZ
        end
    end
    --Check the final node that line B hits
    node = nodes[index]
    if not node or node[IDX_BLOCKED] then
        return false
    end

    return true
end
PathLib.IsLineTraversable = IsLineTraversable


--Calculate whether a rectangle between two points includes only unblocked nodes.
function PathLib.IsAreaUnblocked(mesh: PathMesh, min: Vector3, max: Vector3, includePartial: boolean?): boolean
    local nodes = mesh.Nodes

    min, max = AlignAreaToGrid(min, max, includePartial)

    for z = min.Z, max.Z, GRID_COORD_SPACING do
        local index = CoordToIndex(min.X, z)

        for x = min.X, max.X, GRID_COORD_SPACING do
            local node = nodes[index]

            if (node == nil) or node[IDX_BLOCKED] then
                return false
            end
            index += DeltaCoordToIndex(GRID_COORD_SPACING, 0)
        end
    end
    return true
end


--Calculate whether a point is reachable from the finish point used to calculate costs
function PathLib.IsPointReachable(mesh: PathMesh, point: Vector3): boolean
    local nodePos = ToPathGridRound(point)
    local node = mesh.Nodes[CoordToIndex(nodePos.X, nodePos.Z)]

    return (node ~= nil) and not node[IDX_BLOCKED] and (node[IDX_TOTALCOST] ~= math.huge)
end


--Delete a range of elements from an array
local function DeleteArrayRange<T>(t: {T}, minIndex: number, maxIndex: number): ()
    table.move(t, maxIndex+1, #t+(maxIndex-minIndex+1), minIndex)
end


--Search in both directions along a path to see if straight-line movement can eliminate intermediate nodes
local function TrySimplifyCorner(mesh: PathMesh, path: PathList, cornerIndex: number): (number, number)
    local pathLen = #path
    local p = cornerIndex
    local n = cornerIndex

    --March both directions (towards start and finish of path) simultaneously as far as it's unblocked
    while (p > 1) and (n < pathLen) do
        p -= 1
        n += 1
        if not IsLineTraversable(mesh, path[p], path[n]) then
            p += 1
            n -= 1
            break
        end
    end
    if p == n then
        --The corner couldn't be eliminated,
        return cornerIndex, cornerIndex
    end

    --If the previous loop changed p and n, so the corner node can be eliminated
    --check whether even more nodes can be eliminated in either direction
    if p ~= n then
        --March next point forwards (towards finish) as far as it's unblocked
        while n < pathLen do
            n += 1
            if not IsLineTraversable(mesh, path[p], path[n]) then
                n -= 1
                break
            end
        end

        --March previous point backwards (towards start) as far as it's unblocked
        while p > 1 do
            p -= 1
            if not IsLineTraversable(mesh, path[p], path[n]) then
                p += 1
                break
            end
        end

        --Return the two nodes that must be kept - everything between them can be eliminated
        return p, n
    end
    --The corner couldn't be eliminated,
    return cornerIndex, cornerIndex
end


--[[
Simplify the path by using straight lines to skip nodes where possible.
The resulting path may not closely follow the original path, but only traverses unblocked nodes.
--]]
function PathLib.SimplifyPath(mesh: PathMesh, path: PathList)
    --Remove corner nodes that can be bypassed by straight line movement
    --These corners are typically in open space and just a limitation of 4-direction movement
    local i = 2

    while i < #path do
        local currNode = path[i]

        --Adjecent nodes match in X or Z. If how 3 consecutive nodes match differs, it's a corner
        if (path[i-1].X == currNode.X) ~= (currNode.X == path[i+1].X) then
            local p, n = TrySimplifyCorner(mesh, path, i)
            if (p+1 < n) then
                DeleteArrayRange(path, p+1, n-1)
                i = p --+1 will be added below so it continues with the node "n"
            end
        end

        i += 1
    end

    --Remove runs of nodes in a horizontal or vertical line to reduce waypoint count
    i = 1
    while i < #path do
        local currNode = path[i]
        local n = i + 1

        if(currNode.X == path[n].X) then
            --Advance through intermediate nodes in the straight X line
            while (n < #path) and (currNode.X == path[n+1].X) do
                n += 1
            end

        elseif (currNode.Z == path[n].Z) then
            --Advance through intermediate nodes in the straight Z line
            while (n < #path) and (currNode.Z == path[n+1].Z) do
                n += 1
            end
        end

        --If the line is more than 2 nodes, delete all the middle nodes
        if (i + 1 < n) then
            DeleteArrayRange(path, i+1, n-1)
        end

        i += 1
    end
    return path
end


return table.freeze(PathLib)