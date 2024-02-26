--[[
Copyright Extrarius 2023, 2024
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
local IDX_TOTALCOST = 4
local IDX_BLOCKED = 5
local IDX_NEXTNODE = 6

type PathNode = {any} --Unfortuantely heterogeneous arrays not yet supported in type system
--{Neighbors: {PathNode}, Position: Vector3, Index: number, TotalCost: number, Blocked: boolean, NextNode: PathNode?}
export type PathMesh = {
    Nodes: {[number]: PathNode},
    SortedNodes: {PathNode},
    IsFinalized: boolean,
    FinalizeSeed: number,
    GoalNodes: {PathNode},
    LineCache: {[number]: {[number]: boolean}},
    SimplifyCache: {[Vector3]: Vector3},
    FrontWaveTable: {[number]: PathNode},
    NextWaveTable: {[number]: PathNode},
}
export type PathList = {Vector3}

--Path nodes are spaced every other stud
local GRID_COORD_SPACING = 6
PathLib.GRID_COORD_SPACING = GRID_COORD_SPACING
--Path nodes are centerd in each grid square
local GRID_COORD_OFFSET = 3
PathLib.GRID_COORD_OFFSET = GRID_COORD_OFFSET

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
    return math.round((coord - GRID_COORD_OFFSET) * (1 / GRID_COORD_SPACING)) * GRID_COORD_SPACING + GRID_COORD_OFFSET
end
local function ToPathGridRound(coord: Vector3): Vector3
    return Vector3.new(ToPathGridRoundSingle(coord.X), 0, ToPathGridRoundSingle(coord.Z))
end
PathLib.ToPathGridRound = ToPathGridRound


--Function to snap coordinates to pathing grid by flooring
local function ToPathGridFloorSingle(coord: number): number
    return math.floor((coord - GRID_COORD_OFFSET) * (1 / GRID_COORD_SPACING)) * GRID_COORD_SPACING + GRID_COORD_OFFSET
end
local function ToPathGridFloor(coord: Vector3): Vector3
    return Vector3.new(ToPathGridFloorSingle(coord.X), 0, ToPathGridFloorSingle(coord.Z))
end
PathLib.ToPathGridFloor = ToPathGridFloor


--Function to snap coordinates to pathing grid by ceiling
local function ToPathGridCeilSingle(coord: number): number
    return math.ceil((coord - GRID_COORD_OFFSET) * (1 / GRID_COORD_SPACING)) * GRID_COORD_SPACING + GRID_COORD_OFFSET
end
local function ToPathGridCeil(coord: Vector3): Vector3
    return Vector3.new(ToPathGridCeilSingle(coord.X), 0, ToPathGridCeilSingle(coord.Z))
end
PathLib.ToPathGridCeil = ToPathGridCeil


--Function to test whether a coordinate is on the grid
local function IsOnPathGridSingle(coord: number): boolean
    return (coord % GRID_COORD_SPACING) == GRID_COORD_OFFSET
end
local function IsOnPathGrid(coord: Vector3): boolean
    return IsOnPathGridSingle(coord.X) and IsOnPathGridSingle(coord.Z)
end
PathLib.IsOnPathGrid = IsOnPathGrid


--Function to fit an area to the grid
local function AlignAreaToGrid(min: Vector3, max: Vector3, includePartial: boolean?): (Vector3, Vector3)
    local x0 = min.X
    local x1 = max.X
    local z0 = min.Z
    local z1 = max.Z
    local minX = math.min(x0, x1)
    local minZ = math.min(z0, z1)
    local maxX = math.max(x0, x1)
    local maxZ = math.max(z0, z1)
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
        SortedNodes = {},
        IsFinalized = false,
        FinalizeSeed = seed or os.time(),
        GoalNodes = {},
        LineCache = {},
        SimplifyCache = {},
        FrontWaveTable = {},
        NextWaveTable = {},
    }
end


--Clone a mesh. Finalizes the nodes if the original is finialized
function PathLib.Clone(oldMesh: PathMesh, skipFinalize: boolean?): PathMesh

    local newMesh: PathMesh = {
        Nodes = {},
        SortedNodes = table.create(#oldMesh.SortedNodes),
        IsFinalized = false,
        FinalizeSeed = oldMesh.FinalizeSeed,
        GoalNodes = {},
        LineCache = {},
        SimplifyCache = {},
        FrontWaveTable = {},
        NextWaveTable = {},
    }
    local oldNodes = oldMesh.Nodes
    local oldSorted = oldMesh.SortedNodes
    local newNodes = newMesh.Nodes
    local newSorted = newMesh.SortedNodes

    for index, oldNode in oldNodes do
        local newNode = table.clone(oldNode)

        newNode[IDX_NEIGHBORS] = table.create(4)
        newNode[IDX_NEXTNODE] = nil
        newNodes[index] = newNode
    end
    for _, oldGoal in oldMesh.GoalNodes do
        table.insert(newMesh.GoalNodes, newNodes[oldGoal[IDX_INDEX]])
    end
    --Copy sorted nodes from sorted nodes, so the new one is sorted if the old one is
    for _, oldNode in oldSorted do
        table.insert(newSorted, newNodes[oldNode[IDX_INDEX]])
    end

    if oldMesh.IsFinalized and not skipFinalize then
        --Connect each node to its 4-way neighbors
        for index, current in newNodes do
            local neighbors: {PathNode} = current[IDX_NEIGHBORS]
            local oldNeighbors: {PathNode} = oldNodes[index][IDX_NEIGHBORS]

            for i, neighbor in oldNeighbors do
                neighbors[i] = newNodes[neighbor[IDX_INDEX]]
            end
            --The post-shuffled order was copied
            table.freeze(neighbors)
        end

        newMesh.IsFinalized = true
        table.freeze(newNodes)
        table.freeze(newSorted)
    end

    return newMesh
end


--Add nodes to a mesh to cover a specified region
function PathLib.AddNodes(mesh: PathMesh, min: Vector3, max: Vector3, includePartial: boolean?): ()
    local nodes = mesh.Nodes
    local sorted = mesh.SortedNodes

    min, max = AlignAreaToGrid(min, max, includePartial)

    for z = min.Z, max.Z, GRID_COORD_SPACING do
        local index = CoordToIndex(min.X, z)

        for x = min.X, max.X, GRID_COORD_SPACING do
            if nodes[index] == nil then
                local node: PathNode = {
                    table.create(4),               --Neighbors
                    Vector3.new(x, 0, z),          --Position
                    index,                         --Index
                    math.huge,                     --TotalCost
                    false,                         --Blocked
                    nil,                           --NextNode
                }
                nodes[index] = node
                table.insert(sorted, node)
            end
            index += DeltaCoordToIndex(GRID_COORD_SPACING, 0)
        end
    end
end


--Fisher-Yates fair shuffle
local function shuffle<T>(arr: {T}): {T}
    local arrLen = #arr
    for i = 1, arrLen-1 do
        local j = math.random(i, arrLen)
        arr[i], arr[j] = arr[j], arr[i]
    end
    return arr
end


--Do final processing on each node in the mesh
function PathLib.FinalizeMesh(mesh: PathMesh): ()
    local nodes = mesh.Nodes
    local sorted = mesh.SortedNodes
    local seed = mesh.FinalizeSeed
    local neighbor: PathNode?

    --Make the sorted nodes actually sorted (by index)
    table.sort(sorted, function (lhs, rhs) return lhs[IDX_INDEX] < rhs[IDX_INDEX] end)

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

        math.randomseed(seed + index)
        shuffle(neighbors)

        table.freeze(neighbors)
    end

    mesh.IsFinalized = true
    table.freeze(nodes)
    table.freeze(sorted)
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
    --Clear the cache for line testing since it's invalid
    for _, cache in mesh.LineCache do
        table.clear(cache)
    end
    --Clear the cache for path simplification since it's now invalid
    table.clear(mesh.SimplifyCache)
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
    --Clear the cache for line testing since it's invalid
    for _, cache in mesh.LineCache do
        table.clear(cache)
    end
    --Clear the cache for path simplification since it's now invalid
    table.clear(mesh.SimplifyCache)
end


--Save the current blocked state of every node in the mesh to a buffer
local function SaveBlockedStates(mesh: PathMesh): buffer
    local sorted = mesh.SortedNodes
    local bitmap = buffer.create(4 * ((#sorted + 31) // 32))

    --Pack the state bits into a number
    local value = 0
    local bits = 0
    local offset = 0

    for _, node in sorted do
        value *= 2
        if node[IDX_BLOCKED] then
            value += 1
        end

        --Flush the accumulated bits to the buffer when there are enough
        bits += 1
        if bits == 32 then
            buffer.writeu32(bitmap, offset, value)
            value = 0
            bits = 0
            offset += 4
        end
    end

    --If any bits haven't been flushed to the buffer, do so now
    if bits > 0 then
        value = bit32.lshift(value, 32 - bits)
        buffer.writeu32(bitmap, offset, value)
    end

    return bitmap
end
PathLib.SaveBlockedStates = SaveBlockedStates


--Restore the blocked state of every node in the mesh from a buffer
local function RestoreBlockedStates(mesh: PathMesh, bitmap: buffer)
    local sorted = mesh.SortedNodes

    --Unpack the states from the buffer
    local value = 0
    local bits = 0
    local offset = 0

    for _, node in sorted do
        --If no unpacked bits are left, retrieve more from the buffer
        if bits == 0 then
            value = buffer.readu32(bitmap, offset)
            bits = 32
            offset += 4
        end

        --The high bit matches the blocked state
        if value >= 0x80000000 then
            value -= 0x80000000
            node[IDX_BLOCKED] = true
        else
            node[IDX_BLOCKED] = false
        end

        value *= 2
        bits -= 1
    end
end
PathLib.RestoreBlockedStates = RestoreBlockedStates


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


--Update the costs so paths can be found from anywhere to any goal
local function UpdateMeshCosts(mesh: PathMesh, goalNodes: {PathNode}): boolean
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

    mesh.GoalNodes = table.clone(goalNodes)

    --Do a full expansion of nodes in waves until all unblocked nodes have been assigned the minimum cost to get there
    local frontWave: {[number]: PathNode} = mesh.FrontWaveTable
    local nextWave: {[number]: PathNode} = mesh.NextWaveTable

    for _, goalNode in goalNodes do
        frontWave[goalNode[IDX_INDEX]] = goalNode
        goalNode[IDX_TOTALCOST] = 0
        --Goal nodes point to themselves for simplicity
        goalNode[IDX_NEXTNODE] = goalNode
    end

    while next(frontWave) ~= nil do
        for index, current in frontWave do
            local TotalCost = current[IDX_TOTALCOST] + 1

            for _, neighbor in current[IDX_NEIGHBORS] do
                if (not neighbor[IDX_BLOCKED]) and (TotalCost < neighbor[IDX_TOTALCOST]) then
                    neighbor[IDX_TOTALCOST] = TotalCost
                    nextWave[neighbor[IDX_INDEX]] = neighbor
                end
            end
        end
        frontWave, nextWave = nextWave, frontWave
        table.clear(nextWave)
    end

    return true
end


--Update the costs so paths can be found from anywhere to any of the goals
function PathLib.UpdateMeshCosts(mesh: PathMesh, goals: {Vector3}): boolean
    local goalNodes: {PathNode} = table.create(#goals)
    for ii, goal in goals do
        local goalNodePos = ToPathGridRound(goal)
        local goalNode = mesh.Nodes[CoordToIndex(goalNodePos.X, goalNodePos.Z)]

        if not ValidateNode("Goal", goal, goalNode) then
            return false
        end
        goalNodes[ii] = goalNode
    end

    return UpdateMeshCosts(mesh, goalNodes)
end


--Find a path from the given start node to any goal node using the already-computed costs
local function FindPathUsingCosts(mesh: PathMesh, startNode: PathNode): PathList?
    --Goal nodes are required to calculate the next nodes
    if mesh.GoalNodes[1] == nil then
        CustomError(1, "Attempt to find path before setting goal nodes with UpdateMeshCosts!")
        return nil
    end

    --If the start node wasn't visited, no path can exist
    if startNode[IDX_TOTALCOST] == math.huge then
        CustomWarn("FindPathUsingCosts with start node (", startNode[IDX_POSITION], ") that is unreachable")
        return nil
    end

    --Follow the cheapest nodes from start finish
    --Note that since start has a valid TotalCost, it was visited so a path exists
    --That means this code doesn't need to check for blocked or unvisited neighbors because better ones will exist
    local path: {PathNode} = {startNode}
    local pathLen = 1
    while path[pathLen][IDX_TOTALCOST] ~= 0 do
        local current = path[pathLen]
        local bestNeighbor: PathNode? = current[IDX_NEXTNODE]

        if (bestNeighbor == nil) then
            local bestCost = math.huge

            --If the node isn't blocked and was visited, it's reachable so has at least one good neighbor
            for _, neighbor in current[IDX_NEIGHBORS] do
                local neighborCost: number = neighbor[IDX_TOTALCOST]
                if neighborCost < bestCost then
                    bestNeighbor = neighbor
                    bestCost = neighborCost
                end
            end
            current[IDX_NEXTNODE] = bestNeighbor
        end
        --If a valid neighbor isn't found, no reason to keep searching (should never happen)
        if (bestNeighbor == nil) then
            local goals = {}
            for _, node in mesh.GoalNodes do
                table.insert(goals, "(" .. tostring(node[IDX_POSITION]) .. ")")
            end
            CustomError(1, "FindPathUsingCosts failed to find path between reachable nodes! Start (", startNode[IDX_POSITION], ") to Goals {", table.concat(goals, ", "), "}")
            return nil
        end
        --Record the neighbor
        pathLen += 1
        path[pathLen] = bestNeighbor
    end

    --Store the position of each node used in the correct (reversed, so start to finish) order
    local result: PathList = table.create(pathLen)

    for i, node in path do
        result[i] = node[IDX_POSITION]
    end

    return result
end


--Find a path between the given locations using the already-computed costs
function PathLib.FindPathUsingCosts(mesh: PathMesh, start: Vector3): PathList?
    --Look up the start and finish locations in the node grid
    local startNodePos = ToPathGridRound(start)

    local startNode = mesh.Nodes[CoordToIndex(startNodePos.X, startNodePos.Z)]

    if not ValidateNode("Start", start, startNode) then
        return nil
    end

    return FindPathUsingCosts(mesh, startNode)
end


--[[
    Calculate whether a line between two points includes only unblocked nodes.
    NOTE: The points need not be aligned to the pathing grid!
    Returns false iff the line crosses any grid coordinate with no node or a node marked blocked.

    Based on "A Fast Voxel Traversal Algorithm for Ray Tracing" by John Amanatides and Andrew Woo
    Extended to be a kind of "circle cast" against the path grid by tracing two lines.
--]]
local function RawIsLineTraversable(mesh: PathMesh, start: Vector3, finish: Vector3): boolean
    --Radius can't exceed (or maybe equal?) GRID_COORD_SPACING/2 or nodes might be missed
    local radius = GRID_COORD_SPACING/2 - 0.00001
    local nodes = mesh.Nodes
    local node: PathNode

    local delta = finish - start
    local deltaX = delta.X
    local deltaZ = delta.Z
    local du = delta.Unit --cache for speed
    --How far to step along the X and Z coordinates to get the next grid coordinate
    local stepX = if deltaX > 0 then GRID_COORD_SPACING else -GRID_COORD_SPACING
    local stepZ = if deltaZ > 0 then GRID_COORD_SPACING else -GRID_COORD_SPACING
    --How to adjust the index to get to the cell in the corresponding direction
    local offsetX = DeltaCoordToIndex(stepX, 0)
    local offsetZ = DeltaCoordToIndex(0, stepZ)

    --Two points opposite a circle by the normal of the line
    local rduX = radius * du.X
    local rduZ = radius * du.Z
    local pAX = start.X + rduZ --Yes, the X has the delta-unit Z added because it's perpendicular to the line
    local pAZ = start.Z - rduX --And vice versa, perpendicular to (X, Z) is (Z, -X)
    local pBX = start.X - rduZ --And the other perpendicular vector to (X, Z) is (-Z, X)
    local pBZ = start.Z + rduX
    --Those coordinates snapped to the grid
    local pAXGrid = ToPathGridRoundSingle(pAX)
    local pAZGrid = ToPathGridRoundSingle(pAZ)
    local pBXGrid = ToPathGridRoundSingle(pBX)
    local pBZGrid = ToPathGridRoundSingle(pBZ)

    local index = CoordToIndex(pAXGrid, pAZGrid)

    --If the line is horizontal or vertical, special processinng is required to avoid division by zero
    if deltaX == 0 then
        --For the loop count the Z endpoint is needed
        local pAEndZ = finish.Z - rduX
        local pAEndZGrid = ToPathGridRoundSingle(pAEndZ)

        --If the two lines representing the circle are in the same cell, only check one cell along the way
        if pAXGrid == pBXGrid then
            for zz = pAZGrid, pAEndZGrid, stepZ do
                node = nodes[index]
                if not node or node[IDX_BLOCKED] then
                    return false
                end
                index += offsetZ
            end
        else
            --Otherwise, check one cell for each line
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
    elseif deltaZ == 0 then
        --For the loop count the X endpoint is needed
        local pAEndX = finish.X + rduZ
        local pAEndXGrid = ToPathGridRoundSingle(pAEndX)

        --If the two lines representing the circle are in the same cell, only check one cell along the way
        if pAZGrid == pBZGrid then
            for xx = pAXGrid, pAEndXGrid, stepX do
                node = nodes[index]
                if not node or node[IDX_BLOCKED] then
                    return false
                end
                index += offsetX
            end
        else
            --Otherwise, check one cell for each line
            local indexDelta
            if pAZGrid < pBZGrid then
                indexDelta = DeltaCoordToIndex(0, GRID_COORD_SPACING)
            else
                indexDelta = DeltaCoordToIndex(0, -GRID_COORD_SPACING)
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
    --The T values at which a grid crossing takes place
    local tMaxXA = (pAXGrid + stepX*0.5 - pAX) / deltaX
    local tMaxZA = (pAZGrid + stepZ*0.5 - pAZ) / deltaZ
    local tMaxXB = (pBXGrid + stepX*0.5 - pBX) / deltaX
    local tMaxZB = (pBZGrid + stepZ*0.5 - pBZ) / deltaZ

    --The change in T a step in each direction adavances.
    local tDeltaX = stepX / deltaX
    local tDeltaZ = stepZ / deltaZ

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
PathLib.UncachedIsLineTraversable = RawIsLineTraversable


--Wrapper of IsLineTraversable that uses a cache for grid-aligned queries
local function CachedIsLineTraversable(mesh: PathMesh, start: Vector3, finish: Vector3): boolean
    if not IsOnPathGrid(start) or not IsOnPathGrid(finish) then
        return RawIsLineTraversable(mesh, start, finish)
    end

    local lowIndex = CoordToIndex(start.X, start.Z)
    local highIndex = CoordToIndex(finish.X, finish.Z)
    if lowIndex > highIndex then
        lowIndex, highIndex = highIndex, lowIndex
    end

    local lineCache = mesh.LineCache
    local cache = lineCache[lowIndex]
    if cache == nil then
        cache = {}
        mesh.LineCache[lowIndex] = cache
    end

    local value = cache[highIndex]
    if value ~= nil then
        return value
    end

    value = RawIsLineTraversable(mesh, start, finish)
    cache[highIndex] = value
    return value
end
PathLib.IsLineTraversable = CachedIsLineTraversable


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


--Calculate whether a point is reachable from any goal point used to calculate costs
function PathLib.IsPointReachable(mesh: PathMesh, point: Vector3): boolean
    local nodePos = ToPathGridRound(point)
    local node = mesh.Nodes[CoordToIndex(nodePos.X, nodePos.Z)]

    return (node ~= nil) and not node[IDX_BLOCKED] and (node[IDX_TOTALCOST] ~= math.huge)
end


--Calculate whether a point has an obstacle covering it
function PathLib.IsPointBlocked(mesh: PathMesh, point: Vector3): boolean
    local nodePos = ToPathGridRound(point)
    local node = mesh.Nodes[CoordToIndex(nodePos.X, nodePos.Z)]

    return (node ~= nil) and not node[IDX_BLOCKED]
end


--Search in both directions along a path to see if straight-line movement can eliminate intermediate nodes
local function TrySimplifyCorner(mesh: PathMesh, path: PathList, cornerIndex: number, minStart: number): (number, number)
    local pathLen = #path
    local p = cornerIndex
    local n = cornerIndex

    --March both directions (towards start and finish of path) simultaneously as far as it's unblocked
    while (p > minStart) and (n < pathLen) do
        p -= 1
        n += 1
        if not CachedIsLineTraversable(mesh, path[p], path[n]) then
            p += 1
            n -= 1
            break
        end
    end
    if p == n then
        --The corner couldn't be eliminated,
        return cornerIndex, cornerIndex
    end

    --The previous loop changed p and n, so the corner node can be eliminated
    --check whether even more nodes can be eliminated in either direction

    --March next point forwards (towards finish) as far as it's unblocked
    while n < pathLen do
        n += 1
        if not CachedIsLineTraversable(mesh, path[p], path[n]) then
            n -= 1
            break
        end
    end

    --March previous point backwards (towards start) as far as it's unblocked
    while p > minStart do
        p -= 1
        if not CachedIsLineTraversable(mesh, path[p], path[n]) then
            p += 1
            break
        end
    end

    --Return the two nodes that must be kept - everything between them can be eliminated
    return p, n
end


--Copy nodes to the simplified path while reducing runs of nodes in a line to just the endpoints
local function SimplifyCopyNodes(path: PathList, first: number, last: number, simplified: PathList)
    local i = first
    while i < last do
        local currNode = path[i]
        local n = i + 1

        table.insert(simplified, currNode)

        if(currNode.X == path[n].X) then
            --Advance through intermediate nodes in the straight X line
            while (n < last) and (currNode.X == path[n+1].X) do
                n += 1
            end

        elseif (currNode.Z == path[n].Z) then
            --Advance through intermediate nodes in the straight Z line
            while (n < last) and (currNode.Z == path[n+1].Z) do
                n += 1
            end
        end
        i = n
    end
    if last >= first then
        table.insert(simplified, path[last])
    end
end


--Eliminate nodes that can be bypassed with straight-line movement
local function SimplifyPass2(mesh: PathMesh, path: PathList): PathList
    local pathLen = #path
    local simplePath = {}
    local i = 1
    while i < pathLen do
        local currNode = path[i]
        table.insert(simplePath, currNode)
        local j = i+2
        while (j <= pathLen) and CachedIsLineTraversable(mesh, currNode, path[j]) do
            j += 1
        end
        j -= 1
        i = j
    end
    table.insert(simplePath, path[i])
    return simplePath
end


--[[
Simplify the path by using straight lines to skip nodes where possible.
The resulting path may not closely follow the original path, but only traverses unblocked nodes.
--]]
function PathLib.SimplifyPath(mesh: PathMesh, path: PathList): PathList
    --Remove corner nodes that can be bypassed by diagonal straight line movement
    --These corners are typically in open space and just a limitation of 4-direction movement
    --Also reduce runs of nodes in a line to just the nodes on the ends
    local simplifyCache = mesh.SimplifyCache
    local i = 2
    local simplePath = {}
    local lastFixedNode = 1
    local useCache = false

    while i < #path do
        local currNode = path[i]

        --If this node is in the cache, the rest of the simplified path is known
        if simplifyCache[currNode] then
            --Copy nodes up to this point to the simplified path
            SimplifyCopyNodes(path, lastFixedNode, i, simplePath)
            useCache = true
            break
        end

        --Adjecent nodes match in X or Z. If how 3 consecutive nodes match differs, it's a corner
        if (path[i-1].X == currNode.X) ~= (currNode.X == path[i+1].X) then
            local p, n = TrySimplifyCorner(mesh, path, i, lastFixedNode)

            if (p+1 < n) then
                SimplifyCopyNodes(path, lastFixedNode, p, simplePath)
                lastFixedNode = n
                i = n - 1 --i gets incremented below, so we continue at i=n
            end
        end
        i += 1
    end

    --Copy remaining nodes on the path if the cache doesn't complete the path
    if not useCache then
        SimplifyCopyNodes(path, lastFixedNode, #path, simplePath)
    end

    --Put the path so far into the cache (before the rest is copied from the cache)
    local prevNode = simplePath[1]
    local simplePathLen = #simplePath
    i = 2
    while i <= simplePathLen do
        local currNode = simplePath[i]
        simplifyCache[prevNode] = currNode
        prevNode = currNode
        i += 1
    end

    --If the next point is in the cache, load the rest from the cache
    if useCache then
        local nextNode = simplifyCache[simplePath[simplePathLen]]
        while nextNode do
            simplePathLen += 1
            simplePath[simplePathLen] = nextNode
            nextNode = simplifyCache[nextNode]
        end
    end

    --Perform a second simplification pass that reduces the number of nodes in many cases
    local extraSimplePath = SimplifyPass2(mesh, simplePath)
    if #extraSimplePath < #simplePath then
        --If the second pass improved the simplification, update the cache
        local extrasSimplePathLen = #extraSimplePath
        prevNode = extraSimplePath[1]
        i = 2
        while i <= extrasSimplePathLen do
            local currNode = extraSimplePath[i]
            simplifyCache[prevNode] = currNode
            prevNode = currNode
            i += 1
        end
    end
    return extraSimplePath
end


return table.freeze(PathLib)