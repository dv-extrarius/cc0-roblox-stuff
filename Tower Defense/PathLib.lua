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

export type PathNode = {[number]: any}
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


--Functions to move coordinates to pathing grid
function PathLib.ToPathGridRound(coord: Vector3): Vector3
    return Vector3.new(
        math.round((coord.X - GRID_COORD_OFFSET) / GRID_COORD_SPACING) * GRID_COORD_SPACING + GRID_COORD_OFFSET,
        0,
        math.round((coord.Z - GRID_COORD_OFFSET) / GRID_COORD_SPACING) * GRID_COORD_SPACING + GRID_COORD_OFFSET
    )
end

function PathLib.ToPathGridFloor(coord: Vector3): Vector3
    return Vector3.new(
        math.floor((coord.X - GRID_COORD_OFFSET) / GRID_COORD_SPACING) * GRID_COORD_SPACING + GRID_COORD_OFFSET,
        0,
        math.floor((coord.Z - GRID_COORD_OFFSET) / GRID_COORD_SPACING) * GRID_COORD_SPACING + GRID_COORD_OFFSET
    )
end

function PathLib.ToPathGridCeil(coord: Vector3): Vector3
    return Vector3.new(
        math.ceil((coord.X - GRID_COORD_OFFSET) / GRID_COORD_SPACING) * GRID_COORD_SPACING + GRID_COORD_OFFSET,
        0,
        math.ceil((coord.Z - GRID_COORD_OFFSET) / GRID_COORD_SPACING) * GRID_COORD_SPACING + GRID_COORD_OFFSET
    )
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
        node[IDX_NEIGHBORS] = {}
    end

    if mesh.IsFinalized then
        PathLib.FinalizeMesh(newMesh)
    end
    return newMesh
end

--Add nodes to a mesh to cover a specified region
function PathLib.AddNodes(mesh: PathMesh, min: Vector3, max: Vector3, expand: boolean?): ()
    local nodes = mesh.Nodes

    if not expand then
        min = PathLib.ToPathGridCeil(min)
        max = PathLib.ToPathGridFloor(max)
    else
        min = PathLib.ToPathGridFloor(min)
        max = PathLib.ToPathGridCeil(max)
    end

    for z = min.Z, max.Z, GRID_COORD_SPACING do
        local index = CoordToIndex(min.X, z)

        for x = min.X, max.X, GRID_COORD_SPACING do
            if nodes[index] == nil then 
                nodes[index] = {
                    {},                      --Neighbors
                    Vector3.new(x, 0, z),    --Position 
                    index,                   --Index
                    math.random() / 10000.0, --RandomWeight 
                    math.huge,               --TotalCost
                    false                    --Blocked
                }
            end
            index += DeltaCoordToIndex(GRID_COORD_SPACING, 0)
        end
    end
end

--Do final processing on each node in the mesh and freeze it
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
    --table.freeze(mesh.Nodes)
end

--Mark all nodes in the specified region as blocked
function PathLib.BlockNodes(mesh: PathMesh, min: Vector3, max: Vector3, expand: boolean?): ()
    local nodes = mesh.Nodes

    if not expand then
        min = PathLib.ToPathGridCeil(min)
        max = PathLib.ToPathGridFloor(max)
    else
        min = PathLib.ToPathGridFloor(min)
        max = PathLib.ToPathGridCeil(max)
    end

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
function PathLib.UnblockNodes(mesh: PathMesh, min: Vector3, max: Vector3, expand: boolean?): ()
    local nodes = mesh.Nodes

    if not expand then
        min = PathLib.ToPathGridCeil(min)
        max = PathLib.ToPathGridFloor(max)
    else
        min = PathLib.ToPathGridFloor(min)
        max = PathLib.ToPathGridCeil(max)
    end

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

--Find a path between the given locations using the path mesh
function PathLib.FindPath(mesh: PathMesh, start: Vector3, finish: Vector3): PathList?
    --Look up the start and finish locations in the node grid
    start = PathLib.ToPathGridRound(start)
    finish = PathLib.ToPathGridRound(finish)

    local startNode = mesh.Nodes[CoordToIndex(start.X, start.Z)]
    local finishNode = mesh.Nodes[CoordToIndex(finish.X, finish.Z)]

    if startNode == nil then
        warn("Start point of (", start, ") is outside path nodes")
        return nil
    elseif startNode[IDX_BLOCKED] then
        warn("Start node at (", start, ") is blocked!")
        return nil
    end
    if finishNode == nil then
        warn("Finish point of (", finish, ") is outside path nodes")
        return nil
    elseif finishNode[IDX_BLOCKED] then
        warn("Start node at (", finish, ") is blocked!")
        return nil
    end

    --Reset costs to maximum value
    for _, node in mesh.Nodes do
        node[IDX_TOTALCOST] = math.huge
    end

    --Do a full expansion of nodes in waves until all unblocked nodes have been assigned the minimum cost to get there
    local frontWave: {[number]: PathNode} = {}
    local nextWave: {[number]: PathNode} = {}

    frontWave[startNode[IDX_INDEX]] = startNode
    startNode[IDX_TOTALCOST] = 0

    while next(frontWave) ~= nil do
        for _, current in frontWave do
            local TotalCost = current[IDX_TOTALCOST] + 1 + current[IDX_RANDOMWEIGHT]

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

    --If the finish node wasn't visited, no path can exist
    if finishNode[IDX_TOTALCOST] == math.huge then
        return nil
    end

    --Follow the cheapest nodes from finish to start
    local path: {PathNode} = {finishNode}
    while path[#path] ~= startNode do
        local current = path[#path]
        local bestNeighbor: PathNode? = nil

        for _, neighbor in current[IDX_NEIGHBORS] do
            if (not neighbor[IDX_BLOCKED]) and ((bestNeighbor == nil) or (neighbor[IDX_TOTALCOST] < bestNeighbor[IDX_TOTALCOST])) then
                bestNeighbor = neighbor
            end
        end

        --If a valid neighbor isn't found, no reason to keep searching
        if (bestNeighbor == nil) or table.find(path, bestNeighbor) then
            break
        end
        --Record the neighbor
        table.insert(path, bestNeighbor)
    end
    local pathLen = #path

    --If the path didn't connect to the start node, a path doesn't exist
    if path[pathLen] ~= startNode then
        return nil
    end

    --Store the position of each node used in the correct (reversed, so start to finish) order
    local result: PathList = table.create(pathLen)

    for i = 1, pathLen do
        result[i] = path[pathLen - i + 1][IDX_POSITION]
    end

    return result
end

return table.freeze(PathLib)
