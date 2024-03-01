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
export type MeshBlockedState = buffer

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
        error(table.concat(args, " "), level + 1)
    end
end


--Convert a coordinate to a node index
local function CoordToIndex(x: number, z: number): number
    return (x + (GRID_INDEX_MUL / 2)) + ((z + (GRID_INDEX_MUL / 2)) * GRID_INDEX_MUL)
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
local function AlignAreaToGrid(minCorner: Vector3, maxCorner: Vector3, includePartial: boolean?): (Vector3, Vector3)
    local x0 = minCorner.X
    local x1 = maxCorner.X
    local z0 = minCorner.Z
    local z1 = maxCorner.Z
    local minCornerX = math.min(x0, x1)
    local minCornerZ = math.min(z0, z1)
    local maxCornerX = math.max(x0, x1)
    local maxCornerZ = math.max(z0, z1)
    if not includePartial then
        return Vector3.new(ToPathGridCeilSingle(minCornerX), 0, ToPathGridCeilSingle(minCornerZ)), Vector3.new(ToPathGridFloorSingle(maxCornerX), 0, ToPathGridFloorSingle(maxCornerZ))
    else
        return Vector3.new(ToPathGridFloorSingle(minCornerX), 0, ToPathGridFloorSingle(minCornerZ)), Vector3.new(ToPathGridCeilSingle(maxCornerX), 0, ToPathGridCeilSingle(maxCornerZ))
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
        GoalNodes = table.create(#oldMesh.GoalNodes),
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
        for index, newNode in newNodes do
            local newNeighbors: {PathNode} = newNode[IDX_NEIGHBORS]
            local oldNeighbors: {PathNode} = oldNodes[index][IDX_NEIGHBORS]

            for i, neighbor in oldNeighbors do
                newNeighbors[i] = newNodes[neighbor[IDX_INDEX]]
            end
            --The post-shuffled order was copied, don't reshuffle
            table.freeze(newNeighbors)
        end

        newMesh.IsFinalized = true
        table.freeze(newNodes)
        table.freeze(newSorted)
    end

    return newMesh
end


--Add nodes to a mesh to cover a specified region
function PathLib.AddNodes(mesh: PathMesh, minCorner: Vector3, maxCorner: Vector3, includePartial: boolean?): ()
    local nodes = mesh.Nodes
    local sorted = mesh.SortedNodes

    minCorner, maxCorner = AlignAreaToGrid(minCorner, maxCorner, includePartial)
    local minCornerX = minCorner.X
    local maxCornerX = maxCorner.X

    for z = minCorner.Z, maxCorner.Z, GRID_COORD_SPACING do
        local index = CoordToIndex(minCornerX, z)

        for x = minCornerX, maxCornerX, GRID_COORD_SPACING do
            if not nodes[index] then
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
    for i = 1, arrLen - 1 do
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
    for index, currentNode in nodes do
        local neighbors: {PathNode} = currentNode[IDX_NEIGHBORS]

        neighbor = nodes[index + DeltaCoordToIndex(-GRID_COORD_SPACING, 0)]
        if neighbor then
            table.insert(neighbors, neighbor)
        end

        neighbor = nodes[index + DeltaCoordToIndex( GRID_COORD_SPACING, 0)]
        if neighbor then
            table.insert(neighbors, neighbor)
        end

        neighbor = nodes[index + DeltaCoordToIndex(0, -GRID_COORD_SPACING)]
        if neighbor then
            table.insert(neighbors, neighbor)
        end

        neighbor = nodes[index + DeltaCoordToIndex(0,  GRID_COORD_SPACING)]
        if neighbor then
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

local function ResetCachesForBlockChange(mesh: PathMesh)
    --Clear the cache for line testing since it's invalid
    for _, cache in mesh.LineCache do
        table.clear(cache)
    end
    --Clear the cache for path simplification since it's now invalid
    table.clear(mesh.SimplifyCache)
end


--Mark all nodes in the specified region as blocked
function PathLib.BlockArea(mesh: PathMesh, minCorner: Vector3, maxCorner: Vector3, includePartial: boolean?): ()
    local nodes = mesh.Nodes

    minCorner, maxCorner = AlignAreaToGrid(minCorner, maxCorner, includePartial)
    local minCornerX = minCorner.X
    local maxCornerX = maxCorner.X

    for z = minCorner.Z, maxCorner.Z, GRID_COORD_SPACING do
        local index = CoordToIndex(minCornerX, z)

        for x = minCornerX, maxCornerX, GRID_COORD_SPACING do
            local node = nodes[index]

            if node then
                node[IDX_BLOCKED] = true
            end
            index += DeltaCoordToIndex(GRID_COORD_SPACING, 0)
        end
    end
    ResetCachesForBlockChange(mesh)
end


--Mark all nodes in the specified region as unblocked
function PathLib.UnblockArea(mesh: PathMesh, minCorner: Vector3, maxCorner: Vector3, includePartial: boolean?): ()
    local nodes = mesh.Nodes

    minCorner, maxCorner = AlignAreaToGrid(minCorner, maxCorner, includePartial)
    local minCornerX = minCorner.X
    local maxCornerX = maxCorner.X

    for z = minCorner.Z, maxCorner.Z, GRID_COORD_SPACING do
        local index = CoordToIndex(minCornerX, z)

        for x = minCornerX, maxCornerX, GRID_COORD_SPACING do
            local node = nodes[index]

            if node then
                node[IDX_BLOCKED] = false
            end
            index += DeltaCoordToIndex(GRID_COORD_SPACING, 0)
        end
    end
    ResetCachesForBlockChange(mesh)
end


--Save the current blocked state of every node in the mesh to a buffer
local function SaveBlockedStates(mesh: PathMesh): MeshBlockedState
    local sorted = mesh.SortedNodes
    local state = buffer.create(4 * ((#sorted + 31) // 32))

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
            buffer.writeu32(state, offset, value)
            value = 0
            bits = 0
            offset += 4
        end
    end

    --Flush any bits still in the buffer
    if bits > 0 then
        value = bit32.lshift(value, 32 - bits)
        buffer.writeu32(state, offset, value)
    end

    return state
end
PathLib.SaveBlockedStates = SaveBlockedStates


--Restore the blocked state of every node in the mesh from a buffer
local function RestoreBlockedStates(mesh: PathMesh, state: MeshBlockedState): boolean
    local sorted = mesh.SortedNodes
    local requiredLength = 4 * ((#sorted + 31) // 32)
    local stateLength = buffer.len(state)

    if stateLength ~= requiredLength then
        CustomWarn("Expected state length of ", requiredLength, ", but received state length is ", stateLength)
        if stateLength < requiredLength then
            return false
        end
    end

    --Unpack the states from the buffer
    local value = 0
    local bits = 0
    local offset = 0

    for _, node in sorted do
        --If no unpacked bits are left, retrieve more from the buffer
        if bits == 0 then
            value = buffer.readu32(state, offset)
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

    ResetCachesForBlockChange(mesh)
    return true
end
PathLib.RestoreBlockedStates = RestoreBlockedStates


--Utility to print warnings for invalid start/finish locations
local function ValidateNode(label: string, location: Vector3, node: PathNode): boolean
    if not node then
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

    while next(frontWave) do
        for index, currentNode in frontWave do
            local TotalCost = currentNode[IDX_TOTALCOST] + 1

            for _, neighbor in currentNode[IDX_NEIGHBORS] do
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
    for i, goal in goals do
        local goalNodePos = ToPathGridRound(goal)
        local goalNode = mesh.Nodes[CoordToIndex(goalNodePos.X, goalNodePos.Z)]

        if not ValidateNode("Goal", goal, goalNode) then
            return false
        end
        goalNodes[i] = goalNode
    end

    return UpdateMeshCosts(mesh, goalNodes)
end


--Find a path from the given start node to any goal node using the already-computed costs
local function FindPathUsingCosts(mesh: PathMesh, startNode: PathNode): PathList?
    --Goal nodes are required to calculate the next nodes
    if not mesh.GoalNodes[1] then
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
    local path = {startNode}
    local pathLen = 1
    local currentNode = startNode
    while currentNode[IDX_TOTALCOST] ~= 0 do
        local bestNeighbor: PathNode? = currentNode[IDX_NEXTNODE]

        if not bestNeighbor then
            local bestCost = math.huge

            --If the node isn't blocked and was visited, it's reachable so has at least one good neighbor
            for _, neighbor in currentNode[IDX_NEIGHBORS] do
                local neighborCost: number = neighbor[IDX_TOTALCOST]
                if neighborCost < bestCost then
                    bestNeighbor = neighbor
                    bestCost = neighborCost
                end
            end
            currentNode[IDX_NEXTNODE] = bestNeighbor
        end

        --If a valid neighbor isn't found, no reason to keep searching (should never happen)
        if not bestNeighbor then
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
        currentNode = bestNeighbor
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
local function IsLineTraversableUncached(mesh: PathMesh, start: Vector3, finish: Vector3): boolean
    --Radius can't exceed (or maybe equal?) GRID_COORD_SPACING/2 or nodes might be missed
    local radius = GRID_COORD_SPACING / 2 - 0.00001
    local nodes = mesh.Nodes
    local node: PathNode

    local startX = start.X
    local startZ = start.Z

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
    local pAX = startX + rduZ --Yes, the X has the delta-unit Z added because it's perpendicular to the line
    local pAZ = startZ - rduX --And vice versa. Perpendicular slope to (X, Z) is (Z, -X)
    local pBX = startX - rduZ --And the other perpendicular slope to (X, Z) is (-Z, X)
    local pBZ = startZ + rduX
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
    local tMaxXA = (pAXGrid + (stepX * 0.5) - pAX) / deltaX
    local tMaxZA = (pAZGrid + (stepZ * 0.5) - pAZ) / deltaZ
    local tMaxXB = (pBXGrid + (stepX * 0.5) - pBX) / deltaX
    local tMaxZB = (pBZGrid + (stepZ * 0.5) - pBZ) / deltaZ

    --The change in T a step in each direction adavances.
    local tDeltaX = stepX / deltaX
    local tDeltaZ = stepZ / deltaZ

    --Step line A between X and Z grid crossings, checking each node along the path
    while (tMaxXA < 1) or (tMaxZA < 1) do
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
    while (tMaxXB < 1) or (tMaxZB < 1) do
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
PathLib.IsLineTraversableUncached = IsLineTraversableUncached


--Wrapper of IsLineTraversable that uses a cache for grid-aligned queries
local function IsLineTraversableCached(mesh: PathMesh, start: Vector3, finish: Vector3): boolean
    if not IsOnPathGrid(start) or not IsOnPathGrid(finish) then
        return IsLineTraversableUncached(mesh, start, finish)
    end

    local lowIndex = CoordToIndex(start.X, start.Z)
    local highIndex = CoordToIndex(finish.X, finish.Z)
    if lowIndex > highIndex then
        lowIndex, highIndex = highIndex, lowIndex
    end

    local lineCache = mesh.LineCache
    local cache = lineCache[lowIndex]
    if not cache then
        cache = {}
        mesh.LineCache[lowIndex] = cache
    end

    local value = cache[highIndex]
    if value then
        return value
    end
    value = IsLineTraversableUncached(mesh, start, finish)
    cache[highIndex] = value
    return value
end
PathLib.IsLineTraversable = IsLineTraversableCached


--Calculate whether a rectangle between two points includes only unblocked nodes.
function PathLib.IsAreaUnblocked(mesh: PathMesh, minCorner: Vector3, maxCorner: Vector3, includePartial: boolean?): boolean
    local nodes = mesh.Nodes

    minCorner, maxCorner = AlignAreaToGrid(minCorner, maxCorner, includePartial)
    local minCornerX = minCorner.X
    local maxCornerX = maxCorner.X

    for z = minCorner.Z, maxCorner.Z, GRID_COORD_SPACING do
        local index = CoordToIndex(minCornerX, z)

        for x = minCornerX, maxCornerX, GRID_COORD_SPACING do
            local node = nodes[index]

            if not node or node[IDX_BLOCKED] then
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

    return node and not node[IDX_BLOCKED] and (node[IDX_TOTALCOST] ~= math.huge)
end


--Calculate whether a point has an obstacle covering it
function PathLib.IsPointBlocked(mesh: PathMesh, point: Vector3): boolean
    local nodePos = ToPathGridRound(point)
    local node = mesh.Nodes[CoordToIndex(nodePos.X, nodePos.Z)]

    return node and node[IDX_BLOCKED]
end


--Search in both directions along a path to see if straight-line movement can eliminate intermediate nodes
local function TrySimplifyCorner(mesh: PathMesh, path: PathList, cornerIndex: number, minStart: number): (number, number)
    local pathLen = #path
    local prevIndex = cornerIndex
    local nextIndex = cornerIndex
    local prevPosition
    local nextPosition

    --March both directions (towards start and finish of path) simultaneously as far as it's unblocked
    while (prevIndex > minStart) and (nextIndex < pathLen) do
        prevIndex -= 1
        nextIndex += 1
        prevPosition = path[prevIndex]
        nextPosition = path[nextIndex]
        if not IsLineTraversableCached(mesh, prevPosition, nextPosition) then
            prevIndex += 1
            nextIndex -= 1
            break
        end
    end
    if prevIndex == nextIndex then
        --The corner couldn't be eliminated,
        return cornerIndex, cornerIndex
    end
    prevPosition = path[prevIndex]
    nextPosition = path[nextIndex]

    --The previous loop changed prevIndex and neext, so the corner node can be eliminated
    --check whether even more nodes can be eliminated in either direction

    --March nextIndex point forwards (towards finish) as far as it's unblocked
    local startN = nextIndex
    nextIndex = pathLen
    for i = startN + 1, pathLen do
        nextPosition = path[i]
        if not IsLineTraversableCached(mesh, prevPosition, nextPosition) then
            nextIndex = i - 1
            nextPosition = path[nextIndex]
            break
        end
    end

    ----March previous point backwards (towards start) as far as it's unblocked
    local startP = prevIndex
    prevIndex = minStart
    for i = startP - 1, minStart, -1 do
        prevPosition = path[i]
        if not IsLineTraversableCached(mesh, prevPosition, nextPosition) then
            prevIndex = i + 1
            --prevPosition = path[prevIndex]
            break
        end
    end

    --Return the two nodes that must be kept - everything between them can be eliminated
    return prevIndex, nextIndex
end


--Copy nodes to the simplified path while reducing runs of nodes in a line to just the endpoints
local function SimplifyCopyNodes(path: PathList, first: number, last: number, simplified: PathList)
    local i = first
    while i < last do
        local currentPosition = path[i]
        local currentPositionX = currentPosition.X
        local currentPositionZ = currentPosition.Z
        local nextIndex = i + 1

        table.insert(simplified, currentPosition)

        if currentPositionX == path[nextIndex].X then
            --Advance through intermediate nodes in the straight X line
            while (nextIndex < last) and (currentPositionX == path[nextIndex + 1].X) do
                nextIndex += 1
            end

        elseif currentPositionZ == path[nextIndex].Z then
            --Advance through intermediate nodes in the straight Z line
            while (nextIndex < last) and (currentPositionZ == path[nextIndex + 1].Z) do
                nextIndex += 1
            end
        end
        i = nextIndex
    end

    if last >= first then
        table.insert(simplified, path[last])
    end
end


--Eliminate nodes that can be bypassed with straight-line movement
local function SimplifyPass2(mesh: PathMesh, path: PathList): PathList
    local pathLen = #path
    local extraSimplePath = {}
    local i = 1

    while i < pathLen do
        local currNode = path[i]
        table.insert(extraSimplePath, currNode)

        --Trace as far ahead as can be reached by a straight line
        local reachableIndex = i + 2
        while (reachableIndex <= pathLen) and IsLineTraversableCached(mesh, currNode, path[reachableIndex]) do
            reachableIndex += 1
        end
        reachableIndex -= 1
        i = reachableIndex
    end
    table.insert(extraSimplePath, path[pathLen])
    return extraSimplePath
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
    local simplePath = {}
    local pathLen = #path
    local lastFixedNode = 1
    local useCache = false

    do
        local i = 2
        while i < pathLen do
            local currentPosition = path[i]
            local currentPositionX = currentPosition.X

            --If this position is in the cache, the rest of the simplified path is known
            if simplifyCache[currentPosition] then
                --Copy nodes up to this point to the simplified path
                SimplifyCopyNodes(path, lastFixedNode, i, simplePath)
                useCache = true
                break
            end

            --Adjecent positions match in X or Z. If how 3 consecutive nodes match differs, it's a corner
            if (path[i - 1].X == currentPositionX) ~= (currentPositionX == path[i + 1].X) then
                local prevIndex, nextIndex = TrySimplifyCorner(mesh, path, i, lastFixedNode)

                if prevIndex + 1 < nextIndex then
                    SimplifyCopyNodes(path, lastFixedNode, prevIndex, simplePath)
                    lastFixedNode = nextIndex
                    i = nextIndex - 1 --i gets incremented below, so we continue at i=n
                end
            end
            i += 1
        end
    end

    --Copy remaining nodes on the path if the cache doesn't complete the path
    if not useCache then
        SimplifyCopyNodes(path, lastFixedNode, pathLen, simplePath)
    end

    --Put the path so far into the cache (before the rest is copied from the cache)
    local prevNode = simplePath[1]
    local simplePathLen = #simplePath
    for i = 2, simplePathLen do
        local currentPosition = simplePath[i]
        simplifyCache[prevNode] = currentPosition
        prevNode = currentPosition
    end

    --If the next point is in the cache, follow the cache to the goal
    if useCache then
        local nextPosition = simplifyCache[simplePath[simplePathLen]]
        while nextPosition do
            simplePathLen += 1
            simplePath[simplePathLen] = nextPosition
            nextPosition = simplifyCache[nextPosition]
        end
    end

    --Perform a second simplification pass that reduces the number of nodes in many cases
    local extraSimplePath = SimplifyPass2(mesh, simplePath)
    local extrasSimplePathLen = #extraSimplePath
    if extrasSimplePathLen < simplePathLen then
        --If the second pass improved the simplification, update the cache
        prevNode = extraSimplePath[1]
        for i = 2, extrasSimplePathLen do
            local currentPosition = extraSimplePath[i]
            simplifyCache[prevNode] = currentPosition
            prevNode = currentPosition
        end
    end

    return extraSimplePath
end


return table.freeze(PathLib)