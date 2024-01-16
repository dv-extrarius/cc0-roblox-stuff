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

local IDX_NEIGHBORS = 1
local IDX_POSITION = 2
local IDX_INDEX = 3
local IDX_RANDOMWEIGHT = 4
local IDX_TOTALCOST = 5
local IDX_BLOCKED = 6

export type PathNode = {any}
--{Neighbors: {PathNode}, Position: Vector3, Index: number, RandomWeight: number, TotalCost: number, Blocked: boolean}
export type PathMesh = {Nodes: {[number]: PathNode}, IsFinalized: boolean}
export type PathList = {Vector3}

local PathLib = {}

--Path nodes are spaced every other stud
local GRID_COORD_SPACING = 2
--Path nodes are on odd-numbered studs
local GRID_COORD_OFFSET = 1

--[[
Because a vector3 used as a key is is based on object identity instead of the stored position,
we have to make our own position-to-key function. Since maps won't be huge and nodes are
limited to integer coordinates, we can pack the 3 coordinates using some bits for each one.
Since there are 52 mantissa bits, we chose floor(52/3) bits per coordinate, resulting in a multiplier
of 2^17 = 131072. We keep room for 3 coordinates just in case - for now, we only use X and Z.
Since we want max negative coordinate to map to 0, we add half the multiplier. This allows maps to span
the region with from -65536 to +65535
]]
local GRID_INDEX_MUL = 131072 


--Convert a coordinate to a node index
local function CoordToIndex(x: number, z: number): number
    return (x + (GRID_INDEX_MUL/2)) + ((z + (GRID_INDEX_MUL/2)) * GRID_INDEX_MUL)
end


--Convert an adjustment to a coordinate to an adjustment to an index
local function DeltaCoordToIndex(x: number, z: number): number
    return (x) + (z * GRID_INDEX_MUL)
end


--Function to snap coordinates to pathing grid by rounding
local function ToPathGridRound(coord: Vector3): Vector3
    return Vector3.new(
        math.round((coord.X - GRID_COORD_OFFSET) / GRID_COORD_SPACING) * GRID_COORD_SPACING + GRID_COORD_OFFSET,
        0,
        math.round((coord.Z - GRID_COORD_OFFSET) / GRID_COORD_SPACING) * GRID_COORD_SPACING + GRID_COORD_OFFSET
    )
end
PathLib.ToPathGridRound = ToPathGridRound


--Function to snap coordinates to pathing grid by flooring
function ToPathGridFloor(coord: Vector3): Vector3
    return Vector3.new(
        math.floor((coord.X - GRID_COORD_OFFSET) / GRID_COORD_SPACING) * GRID_COORD_SPACING + GRID_COORD_OFFSET,
        0,
        math.floor((coord.Z - GRID_COORD_OFFSET) / GRID_COORD_SPACING) * GRID_COORD_SPACING + GRID_COORD_OFFSET
    )
end
PathLib.ToPathGridFloor = ToPathGridFloor


--Function to snap coordinates to pathing grid by ceiling
function ToPathGridCeil(coord: Vector3): Vector3
    return Vector3.new(
        math.ceil((coord.X - GRID_COORD_OFFSET) / GRID_COORD_SPACING) * GRID_COORD_SPACING + GRID_COORD_OFFSET,
        0,
        math.ceil((coord.Z - GRID_COORD_OFFSET) / GRID_COORD_SPACING) * GRID_COORD_SPACING + GRID_COORD_OFFSET
    )
end
PathLib.ToPathGridCeil = ToPathGridCeil


--Function to fit an area to the grid
local function AreaToGrid(min: Vector3, max: Vector3, includePartial: boolean?): (Vector3, Vector3)
    local high = Vector3.new(math.max(max.X, min.X), math.max(max.Y, min.Y), math.max(max.Z, min.Z))
    local low = Vector3.new(math.min(max.X, min.X), math.min(max.Y, min.Y), math.min(max.Z, min.Z))
    if not includePartial then
        return ToPathGridCeil(low), ToPathGridFloor(high)
    else
        return ToPathGridFloor(low), ToPathGridCeil(high)
    end
end


--Create an empty mesh
function PathLib.NewMesh(): PathMesh
    return {Nodes = {}, IsFinalized = false}
end


--Clone a mesh. Finalizes the nodes if the original is finialized
function PathLib.Clone(mesh: PathMesh): PathMesh
    local newMesh: PathMesh = {Nodes = {}, IsFinalized = false}

    for index, current in mesh.Nodes do
        local node = table.clone(current)
        node[IDX_NEIGHBORS] = table.create(4)
    end

    if mesh.IsFinalized then
        PathLib.FinalizeMesh(newMesh)
    end
    return newMesh
end


--Add nodes to a mesh to cover a specified region
function PathLib.AddNodes(mesh: PathMesh, min: Vector3, max: Vector3, includePartial: boolean?): ()
    local nodes = mesh.Nodes

    min, max = AreaToGrid(min, max, includePartial)

    for z = min.Z, max.Z, GRID_COORD_SPACING do
        local index = CoordToIndex(min.X, z)

        for x = min.X, max.X, GRID_COORD_SPACING do
            if nodes[index] == nil then 
                nodes[index] = {
                    table.create(4),            --Neighbors
                    Vector3.new(x, 0, z),       --Position 
                    index,                      --Index
                    (math.random() / 100000.0), --RandomWeight 
                    math.huge,                  --TotalCost
                    false                       --Blocked
                }
            end
            index += DeltaCoordToIndex(GRID_COORD_SPACING, 0)
        end
    end
end


--Do final processing on each node in the mesh
function PathLib.FinalizeMesh(mesh: PathMesh): ()
    local nodes = mesh.Nodes
    local neighbor: PathNode?

    --Connect each node to 4-way neighbors if they exist
    for index, current in nodes do
        neighbor = nodes[index + DeltaCoordToIndex(-GRID_COORD_SPACING, 0)]
        if neighbor ~= nil then
            table.insert(current[IDX_NEIGHBORS], neighbor)
        end

        neighbor = nodes[index + DeltaCoordToIndex( GRID_COORD_SPACING, 0)]
        if neighbor ~= nil then
            table.insert(current[IDX_NEIGHBORS], neighbor)
        end

        neighbor = nodes[index + DeltaCoordToIndex(0, -GRID_COORD_SPACING)]
        if neighbor ~= nil then
            table.insert(current[IDX_NEIGHBORS], neighbor)
        end

        neighbor = nodes[index + DeltaCoordToIndex(0,  GRID_COORD_SPACING)]
        if neighbor ~= nil then
            table.insert(current[IDX_NEIGHBORS], neighbor)
        end

        table.freeze(current[IDX_NEIGHBORS])
    end

    mesh.IsFinalized = true
    table.freeze(mesh.Nodes)
end


--Mark all nodes in the specified region as blocked
function PathLib.BlockNodes(mesh: PathMesh, min: Vector3, max: Vector3, includePartial: boolean?): ()
    local nodes = mesh.Nodes

    min, max = AreaToGrid(min, max, includePartial)

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

    min, max = AreaToGrid(min, max, includePartial)

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
        warn(label, " point of (", location, ") is outside path nodes")
        return false
    elseif node[IDX_BLOCKED] then
        warn(label, " node at (", location, ") is blocked!")
        return false
    end
    return true
end


--Update the costs so paths can be found from anywhere to finishNode
local function UpdateMeshCosts(mesh: PathMesh, finishNode: PathNode): boolean
    --Finding a path requires a finalized mesh
    if not mesh.IsFinalized then
        return false
    end

    --Reset costs to maximum value
    for _, node in mesh.Nodes do
        node[IDX_TOTALCOST] = math.huge
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
        table.clear(nextWave)
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


--Find a path between the given nodes using the already-computed costs
local function FindPathUsingCosts(mesh: PathMesh, startNode: PathNode, finishNode: PathNode): PathList?
    --If the finish node wasn't the origin, the costs can't be used for this path
    if finishNode[IDX_TOTALCOST] ~= 0 then
        return nil
    end

    --If the start node wasn't visited, no path can exist
    if startNode[IDX_TOTALCOST] == math.huge then
        return nil
    end

    --Follow the cheapest nodes from start finish
    local path: {PathNode} = {startNode}
    while path[#path] ~= finishNode do
        local current = path[#path]
        local bestNeighbor: PathNode? = nil

        for _, neighbor in current[IDX_NEIGHBORS] do
            if ((bestNeighbor == nil) or (neighbor[IDX_TOTALCOST] < bestNeighbor[IDX_TOTALCOST])) then
                bestNeighbor = neighbor
            end
        end

        --If a valid neighbor isn't found, no reason to keep searching
        if (bestNeighbor == nil) then
            break
        end
        --Record the neighbor
        table.insert(path, bestNeighbor)
    end
    local pathLen = #path

    --If the path didn't connect to the finish node, a path doesn't exist
    if path[pathLen] ~= finishNode then
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
        warn("Finish node at (", finish, ") was not the finish used to update costs!")
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
    Calculate whether a line between two nodes includes only unblocked nodes.
    The provided coordinates must be aligned to the pathing grid (ToPathGrid*)
    Returns false iff the line crosses any grid coordinate with no node or a node marked blocked.
    
    Based on "A Fast Voxel Traversal Algorithm for Ray Tracing" by John Amanatides and Andrew Woo
    Extended to be a kind of "circle cast" against the path grid by tracing two lines.
--]]
function IsLineUnblocked(mesh: PathMesh, start: Vector3, finish: Vector3): boolean
    --Radius can't exceed 1 or nodes might be missed
    local radius = 0.99
    local nodes = mesh.Nodes
    local node: PathNode

    local delta = finish - start
    --How far to step along the X and Z coordinates to get the next grid coordinate
    local stepX = if delta.X > 0 then GRID_COORD_SPACING else -GRID_COORD_SPACING
    local stepZ = if delta.Z > 0 then GRID_COORD_SPACING else -GRID_COORD_SPACING
    --How to adjust the index to get to the cell in the corresponding direction
    local offsetX = DeltaCoordToIndex(stepX, 0)
    local offsetZ = DeltaCoordToIndex(0, stepZ)

    local index = CoordToIndex(start.X, start.Z)
    --If the line is horizontal or vertical, special processinng is required to avoid division by zero
    if delta.X == 0 then
        for z = start.Z, finish.Z, stepZ do
            node = nodes[index]
            if not node or node[IDX_BLOCKED] then
                return false
            end
            index += offsetZ
        end
        return true
    elseif delta.Z == 0 then
        for x = start.X, finish.X, stepX do
            node = nodes[index]
            if not node or node[IDX_BLOCKED] then
                return false
            end
            index += offsetX
        end
        return true
    end
    
    --The change in T a step in each direction adavances.
    local tDeltaX = stepX / delta.X
    local tDeltaZ = stepZ / delta.Z

    local du = delta.Unit --Cache the coordinate change's unit vector
    
    --Coordinates used for two lines adjusted based on a circle 
    --local pAX = start.X + radius *  du.Z
    --local pAZ = start.Z + radius * -du.X
    --local pBX = start.X + radius * -du.Z
    --local pBZ = start.Z + radius *  du.X
    
    --The T value for the next X and Z crossing for each of the lines
    --local tMaxXA = (start.X + stepX/2 - pAX) / delta.X
    --local tMaxZA = (start.Z + stepZ/2 - pAZ) / delta.Z
    --local tMaxXB = (start.X + stepX/2 - pBX) / delta.X
    --local tMaxZB = (start.Z + stepZ/2 - pBZ) / delta.Z
    
    --Manually simplified expressions to ensure optimality
    local tMaxXA = (stepX/2 - radius *  du.Z) / delta.X
    local tMaxZA = (stepZ/2 - radius * -du.X) / delta.Z
    local tMaxXB = (stepX/2 - radius * -du.Z) / delta.X
    local tMaxZB = (stepZ/2 - radius *  du.X) / delta.Z
    
    --Step line A between X and Z grid crossings, checking each node along the path
    while (tMaxXA <= 1) or (tMaxZA <= 1) do
        node = nodes[index]
        if not node or node[IDX_BLOCKED] then
            return false
        end
        --Advance to the nearest grid crossing, whether in X or Z direction
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
    index = CoordToIndex(start.X, start.Z)    
    while (tMaxXB <= 1) or (tMaxZB <= 1) do
        node = nodes[index]
        if not node or node[IDX_BLOCKED] then
            return false
        end
        --Advance to the nearest grid crossing, whether in X or Z direction
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

--Calculate whether a line between two nodes includes only unblocked nodes.
function PathLib.IsLineUnblocked(mesh: PathMesh, start: Vector3, finish: Vector3): boolean
    --Look up the start and finish locations in the node grid
    local startNodePos = ToPathGridRound(start)
    local finishNodePos = ToPathGridRound(finish)
    
    return IsLineUnblocked(mesh, startNodePos, finishNodePos)
end


return table.freeze(PathLib)