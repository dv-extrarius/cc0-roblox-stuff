--[[
Copyright Extrarius 2023, 2024
This work is marked with CC0 1.0 Universal.
To view a copy of this license, visit http://creativecommons.org/publicdomain/zero/1.0
--
This is an implementation of an essentially 2d pathfinding system. It ignores Y coordinates, and
uses a grid of cells that are manually marked blocked or unblocked by the library's user.

Work In Progress - This is modified from my regular Path-Lib in an attempt to make nice flow fields.
It's not quite there yet.
]]
--!strict
--!optimize 2

local FlowField = {}

--Constants used to index the FieldCell type
local IDX_NEIGHBORS = 1
local IDX_POSITION = 2
local IDX_INDEX = 3
local IDX_TOTALCOST = 4
local IDX_BLOCKED = 5
local IDX_DIRECTION = 6

type FieldCell = {any} --Unfortuantely heterogeneous arrays not yet supported in type system
--{Neighbors: {FieldCell}, Position: Vector3, Index: number, TotalCost: number, Blocked: boolean, Direction: Vector3}
export type Field = {
    Cells: {[number]: FieldCell},
    SortedCells: {FieldCell},
    BorderCells: {[number]: FieldCell},
    IsFinalized: boolean,
    GoalCells: {FieldCell},
    LineCache: {[number]: boolean},
    FrontWaveTable: {[number]: FieldCell},
    NextWaveTable: {[number]: FieldCell},

}
export type FieldBlockedState = buffer

--Field cells are spaced every other stud
local GRID_COORD_SPACING = 6
FlowField.GRID_COORD_SPACING = GRID_COORD_SPACING
--Field cells are centerd in each grid square
local GRID_COORD_OFFSET = 3
FlowField.GRID_COORD_OFFSET = GRID_COORD_OFFSET

--[[
Before I knew Vector3 were value types, I made my own position-to-key system. After discovering
they are value types, a benchmark showed my system is about 5% faster for my cast, so here it is.
Since places won't be huge and cells are limited to integer coordinates, we can pack the coordinates
using some bits for each one. Since there are effectively 53 mantissa bits, I chose floor(53/4) bits
per coordinate, resulting in a multiplier of 2^13 = 8192. I chose to divide by 4 to allow packing
the descripton of a line into a single number, which helps with the line cache.
Thus, pathfinding cells can span the range of coordinates -4096 to +4095, or approximately far enough
]]
local GRID_INDEX_MUL = 8192

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


--Convert a coordinate to a cell index
local function CoordToIndex(x: number, z: number): number
    --TODO: Maybe take advantage of grid size to increase range?
    --Reorganizing terms results in better bytecode:
    --return (x + (GRID_INDEX_MUL / 2)) + ((z + (GRID_INDEX_MUL / 2)) * GRID_INDEX_MUL)
    return x + (z * GRID_INDEX_MUL) + ((GRID_INDEX_MUL / 2) + (GRID_INDEX_MUL / 2) * GRID_INDEX_MUL)
end
FlowField.CoordToIndex = CoordToIndex


--Convert an adjustment to a coordinate to an adjustment to an index
local function DeltaCoordToIndex(x: number, z: number): number
    --TODO: Maybe take advantage of grid size to increase range?
    return (x) + (z * GRID_INDEX_MUL)
end
--Convert an adjustment to an X coordinate to an adjustment to an index
local function DeltaXCoordToIndex(x: number): number
    --TODO: Maybe take advantage of grid size to increase range?
    return (x)
end
--Convert an adjustment to a Z coordinate to an adjustment to an index
local function DeltaZCoordToIndex(z: number): number
    --TODO: Maybe take advantage of grid size to increase range?
    return (z * GRID_INDEX_MUL)
end


--Combine a pair of indexes into a single one describing a line
local function CombineIndexesIntoLine(a: number, b: number): number
    if a <= b then
        return a + (b * (GRID_INDEX_MUL * GRID_INDEX_MUL))
    else
        return b + (a * (GRID_INDEX_MUL * GRID_INDEX_MUL))
    end
end


--Function to snap coordinates to field grid by rounding
local function ToCellGridRoundSingle(coord: number): number
    return math.round((coord - GRID_COORD_OFFSET) * (1 / GRID_COORD_SPACING)) * GRID_COORD_SPACING + GRID_COORD_OFFSET
end
local function ToCellGridRound(coord: Vector3): Vector3
    return Vector3.new(ToCellGridRoundSingle(coord.X), 0, ToCellGridRoundSingle(coord.Z))
end
FlowField.ToCellGridRound = ToCellGridRound


--Function to snap coordinates to field grid by flooring
local function ToCellGridFloorSingle(coord: number): number
    return math.floor((coord - GRID_COORD_OFFSET) * (1 / GRID_COORD_SPACING)) * GRID_COORD_SPACING + GRID_COORD_OFFSET
end
local function ToCellGridFloor(coord: Vector3): Vector3
    return Vector3.new(ToCellGridFloorSingle(coord.X), 0, ToCellGridFloorSingle(coord.Z))
end
FlowField.ToCellGridFloor = ToCellGridFloor


--Function to snap coordinates to field grid by ceiling
local function ToCellGridCeilSingle(coord: number): number
    return math.ceil((coord - GRID_COORD_OFFSET) * (1 / GRID_COORD_SPACING)) * GRID_COORD_SPACING + GRID_COORD_OFFSET
end
local function ToCellGridCeil(coord: Vector3): Vector3
    return Vector3.new(ToCellGridCeilSingle(coord.X), 0, ToCellGridCeilSingle(coord.Z))
end
FlowField.ToCellGridCeil = ToCellGridCeil


--Function to test whether a coordinate is on the grid
local function IsOnPathGridSingle(coord: number): boolean
    return (coord % GRID_COORD_SPACING) == GRID_COORD_OFFSET
end
local function IsOnPathGrid(coord: Vector3): boolean
    return IsOnPathGridSingle(coord.X) and IsOnPathGridSingle(coord.Z)
end
FlowField.IsOnPathGrid = IsOnPathGrid


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
        return Vector3.new(ToCellGridCeilSingle(minCornerX), 0, ToCellGridCeilSingle(minCornerZ)), Vector3.new(ToCellGridFloorSingle(maxCornerX), 0, ToCellGridFloorSingle(maxCornerZ))
    else
        return Vector3.new(ToCellGridFloorSingle(minCornerX), 0, ToCellGridFloorSingle(minCornerZ)), Vector3.new(ToCellGridCeilSingle(maxCornerX), 0, ToCellGridCeilSingle(maxCornerZ))
    end
end
FlowField.AlignAreaToGrid = AlignAreaToGrid


--Initialize FlowField with the provided settings
function FlowField.Configure(config: {EnableDebugMessages: boolean?}): ()
    if config.EnableDebugMessages ~= nil then
        EnableDebugMessages = config.EnableDebugMessages and true or false --cast to boolean
    end
end


--Create an empty field
function FlowField.NewField(): Field
    return {
        Cells = {},
        SortedCells = {},
        BorderCells = {},
        IsFinalized = false,
        GoalCells = {},
        LineCache = {},
        FrontWaveTable = {},
        NextWaveTable = {},
    }
end


--Clone a field. Finalizes the cells if the original is finialized
function FlowField.Clone(oldField: Field, skipFinalize: boolean?): Field

    local newField: Field = {
        Cells = {},
        SortedCells = table.create(#oldField.SortedCells),
        BorderCells = {},
        IsFinalized = false,
        GoalCells = table.create(#oldField.GoalCells),
        LineCache = {},
        FrontWaveTable = {},
        NextWaveTable = {},
    }
    local oldCells = oldField.Cells
    local oldSorted = oldField.SortedCells
    local newCells = newField.Cells
    local newSorted = newField.SortedCells

    for index, oldCell in oldCells do
        local newCell = table.clone(oldCell)

        newCell[IDX_NEIGHBORS] = table.create(4)
        newCells[index] = newCell
    end
    for _, oldGoal in oldField.GoalCells do
        table.insert(newField.GoalCells, newCells[oldGoal[IDX_INDEX]])
    end
    --Copy sorted cells from sorted cells, so the new one is sorted if the old one is
    for _, oldCell in oldSorted do
        table.insert(newSorted, newCells[oldCell[IDX_INDEX]])
    end

    if oldField.IsFinalized and not skipFinalize then
        local newBorderCells = oldField.BorderCells
        newField.BorderCells = newBorderCells
        --Connect each cell to its 4-way neighbors
        for index, newCell in newCells do
            local newNeighbors: {FieldCell} = newCell[IDX_NEIGHBORS]
            local oldNeighbors: {FieldCell} = oldCells[index][IDX_NEIGHBORS]

            for i, neighbor in oldNeighbors do
                local index = neighbor[IDX_INDEX]
                newNeighbors[i] = newCells[index] or newBorderCells[index]
            end

            table.freeze(newNeighbors)
        end

        newField.IsFinalized = true
        table.freeze(newCells)
        table.freeze(newSorted)
    end

    return newField
end


--Add cells to a field to cover a specified region
function FlowField.AddArea(field: Field, minCorner: Vector3, maxCorner: Vector3, includePartial: boolean?): ()
    local cells = field.Cells
    local sorted = field.SortedCells

    minCorner, maxCorner = AlignAreaToGrid(minCorner, maxCorner, includePartial)
    local minCornerX = minCorner.X
    local maxCornerX = maxCorner.X

    for z = minCorner.Z, maxCorner.Z, GRID_COORD_SPACING do
        local index = CoordToIndex(minCornerX, z)

        for x = minCornerX, maxCornerX, GRID_COORD_SPACING do
            if not cells[index] then
                local cell: FieldCell = {
                    table.create(4),               --Neighbors
                    Vector3.new(x, 0, z),          --Position
                    index,                         --Index
                    math.huge,                     --TotalCost
                    false,                         --Blocked
                    Vector3.zero,                  --Direction
                }
                cells[index] = cell
                table.insert(sorted, cell)
            end
            index += DeltaXCoordToIndex(GRID_COORD_SPACING)
        end
    end
end


local frozenEmptyNeighbors: {FieldCell} = table.freeze({})
local borderCellStr = "BorderCell"
local function MakeBorderCell(neighborIndex: number, borderPosition: Vector3): FieldCell
    return table.freeze({
        frozenEmptyNeighbors, --Neighbors
        borderPosition,       --Position
        neighborIndex,        --Index
        math.huge,            --TotalCost
        true,                 --Blocked
        Vector3.zero,         --Direction
        borderCellStr --extra to make them obvious
    } :: FieldCell)
end


--Do final processing on each cell in the field
function FlowField.FinalizeField(field: Field): ()
    local cells = field.Cells
    local borderCells = field.BorderCells
    local sorted = field.SortedCells
    local neighbor: FieldCell?
    local neighborIndex: number

    --Make the sorted cells actually sorted (by index)
    table.sort(sorted, function (lhs, rhs) return lhs[IDX_INDEX] < rhs[IDX_INDEX] end)

    --Connect each cell to 4-way neighbors if they exist and create border cells if they don't
    for index, currentCell in cells do
        local neighbors: {FieldCell} = currentCell[IDX_NEIGHBORS]

        neighborIndex = index + DeltaXCoordToIndex(-GRID_COORD_SPACING)
        neighbor = cells[neighborIndex] or borderCells[neighborIndex]
        if not neighbor then
            neighbor = MakeBorderCell(neighborIndex, currentCell[IDX_POSITION] - GRID_COORD_SPACING*Vector3.xAxis)
            borderCells[neighborIndex] = neighbor
        end
        table.insert(neighbors, neighbor :: FieldCell)


        neighborIndex = index + DeltaXCoordToIndex(GRID_COORD_SPACING)
        neighbor = cells[neighborIndex] or borderCells[neighborIndex]
        if not neighbor then
            neighbor = MakeBorderCell(neighborIndex, currentCell[IDX_POSITION] + GRID_COORD_SPACING*Vector3.xAxis)
            borderCells[neighborIndex] = neighbor
        end
        table.insert(neighbors, neighbor :: FieldCell)


        neighborIndex = index + DeltaZCoordToIndex(-GRID_COORD_SPACING)
        neighbor = cells[neighborIndex] or borderCells[neighborIndex]
        if not neighbor then
            neighbor = MakeBorderCell(neighborIndex, currentCell[IDX_POSITION] - GRID_COORD_SPACING*Vector3.zAxis)
            borderCells[neighborIndex] = neighbor
        end
        table.insert(neighbors, neighbor :: FieldCell)


        neighborIndex = index + DeltaZCoordToIndex(GRID_COORD_SPACING)
        neighbor = cells[neighborIndex] or borderCells[neighborIndex]
        if not neighbor then
            neighbor = MakeBorderCell(neighborIndex, currentCell[IDX_POSITION] + GRID_COORD_SPACING*Vector3.zAxis)
            borderCells[neighborIndex] = neighbor
        end
        table.insert(neighbors, neighbor :: FieldCell)


        table.freeze(neighbors)
    end

    field.IsFinalized = true
    table.freeze(cells)
    table.freeze(borderCells)
    table.freeze(sorted)
end

local function ResetCachesForBlockChange(field: Field)
    --Clear the cache for line testing since blocking changes invalidate it
    table.clear(field.LineCache)
end


--Mark all cells in the specified region as blocked
function FlowField.BlockArea(field: Field, minCorner: Vector3, maxCorner: Vector3, includePartial: boolean?): ()
    local cells = field.Cells

    minCorner, maxCorner = AlignAreaToGrid(minCorner, maxCorner, includePartial)
    local minCornerX = minCorner.X
    local maxCornerX = maxCorner.X

    for z = minCorner.Z, maxCorner.Z, GRID_COORD_SPACING do
        local index = CoordToIndex(minCornerX, z)

        for x = minCornerX, maxCornerX, GRID_COORD_SPACING do
            local cell = cells[index]

            if cell then
                cell[IDX_BLOCKED] = true
            end
            index += DeltaXCoordToIndex(GRID_COORD_SPACING)
        end
    end
    ResetCachesForBlockChange(field)
end


--Mark all cells in the specified region as unblocked
function FlowField.UnblockArea(field: Field, minCorner: Vector3, maxCorner: Vector3, includePartial: boolean?): ()
    local cells = field.Cells

    minCorner, maxCorner = AlignAreaToGrid(minCorner, maxCorner, includePartial)
    local minCornerX = minCorner.X
    local maxCornerX = maxCorner.X

    for z = minCorner.Z, maxCorner.Z, GRID_COORD_SPACING do
        local index = CoordToIndex(minCornerX, z)

        for x = minCornerX, maxCornerX, GRID_COORD_SPACING do
            local cell = cells[index]

            if cell then
                cell[IDX_BLOCKED] = false
            end
            index += DeltaXCoordToIndex(GRID_COORD_SPACING)
        end
    end
    ResetCachesForBlockChange(field)
end


--Save the current blocked state of every cell in the field to a buffer
local function SaveBlockedStates(field: Field): FieldBlockedState
    local sorted = field.SortedCells
    local state = buffer.create(4 * ((#sorted + 31) // 32))

    --Pack the state bits into a number
    local value = 0
    local bits = 0
    local offset = 0

    for _, cell in sorted do
        value *= 2
        if cell[IDX_BLOCKED] then
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
FlowField.SaveBlockedStates = SaveBlockedStates


--Restore the blocked state of every cell in the field from a buffer
local function RestoreBlockedStates(field: Field, state: FieldBlockedState): boolean
    local sorted = field.SortedCells
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

    for _, cell in sorted do
        --If no unpacked bits are left, retrieve more from the buffer
        if bits == 0 then
            value = buffer.readu32(state, offset)
            bits = 32
            offset += 4
        end

        --The high bit matches the blocked state
        if value >= 0x80000000 then
            value -= 0x80000000
            cell[IDX_BLOCKED] = true
        else
            cell[IDX_BLOCKED] = false
        end

        value *= 2
        bits -= 1
    end

    ResetCachesForBlockChange(field)
    return true
end
FlowField.RestoreBlockedStates = RestoreBlockedStates


--Utility to print warnings for invalid start/finish locations
local function ValidateCell(label: string, location: Vector3, cell: FieldCell): boolean
    if not cell then
        CustomWarn(label, " point of (", location, ") is outside field cells")
        return false
    elseif cell[IDX_BLOCKED] then
        CustomWarn(label, " cell at (", location, ") is blocked!")
        return false
    end
    return true
end


--Update the costs so the field points the way to the goal
local function UpdateField(field: Field, goalCells: {FieldCell}): boolean
    --Updating the costs requires a finalized field
    if not field.IsFinalized then
        CustomError(1, "Attempted UpdateField without finalizing the field.")
        return false
    end
    local cells = field.Cells


    --Reset costs to maximum value
    for _, cell in cells do
        cell[IDX_TOTALCOST] = math.huge
        cell[IDX_DIRECTION] = Vector3.zero
    end

    field.GoalCells = table.clone(goalCells)

    --Do a full expansion of cells in waves until all unblocked cells have been assigned the minimum cost to get there
    local frontWave: {[number]: FieldCell} = field.FrontWaveTable
    local nextWave: {[number]: FieldCell} = field.NextWaveTable

    table.clear(frontWave)
    table.clear(nextWave)

    for _, goalCell in goalCells do
        frontWave[goalCell[IDX_INDEX]] = goalCell
        goalCell[IDX_TOTALCOST] = 0
    end

    for index, currentCell in frontWave do
        if currentCell[IDX_TOTALCOST] == math.huge then
            warn("Visiting unexplored cell!?", currentCell[IDX_POSITION], ";", currentCell)
        end
    end

    local blockedDirections: {Vector3} = table.create(3)
    local parentDirections: {Vector3} = table.create(3)
    local parentCenters: {Vector3} = table.create(3)
    while next(frontWave) do
        for index, currentCell in frontWave do
            local totalCost = currentCell[IDX_TOTALCOST] + 1
            local position: Vector3 = currentCell[IDX_POSITION]
            local centerPoint: Vector3 = Vector3.zero
            local numCenters = 0

            if currentCell[IDX_TOTALCOST] == math.huge then
                warn("Visiting unexplored cell!?", currentCell[IDX_POSITION], ";", currentCell)
            end

            --Process each neighbor
            for _, neighbor in currentCell[IDX_NEIGHBORS] do
                local blocked = neighbor[IDX_BLOCKED]
                local neighborIndex = neighbor[IDX_INDEX]

                if blocked then
                    --local blockedCost = totalCost + 2^24
                    --if cells[neighborIndex] and (blockedCost < neighbor[IDX_TOTALCOST]) then
                    --    --Neighbors whose costs can be lowered need to be explored
                    --    neighbor[IDX_TOTALCOST] = blockedCost
                    --    nextWave[neighborIndex] = neighbor
                    --end
                elseif cells[neighborIndex] and (totalCost < neighbor[IDX_TOTALCOST]) then
                    --Neighbors whose costs can be lowered need to be explored
                    neighbor[IDX_TOTALCOST] = totalCost
                    nextWave[neighborIndex] = neighbor
                end
            end
        end
        frontWave, nextWave = nextWave, frontWave
        table.clear(nextWave)
    end

    local sortedCells = table.clone(field.SortedCells)
    table.sort(sortedCells, function(a,b) return a[IDX_TOTALCOST] < b[IDX_TOTALCOST] end)

    for _, currentCell in sortedCells do
        local index = currentCell[IDX_INDEX]
        local myCost = currentCell[IDX_TOTALCOST]
        local totalCost = currentCell[IDX_TOTALCOST] + 1
        local position: Vector3 = currentCell[IDX_POSITION]
        local centerPoint: Vector3 = Vector3.zero
        local numCenters = 0


        for _, neighbor in currentCell[IDX_NEIGHBORS] do
            local blocked = neighbor[IDX_BLOCKED]
            local neighborIndex = neighbor[IDX_INDEX]

            if blocked then
                ----Info about blocked neighbors is needed to make sure this cell doesn't point that way
                local delta = (neighbor[IDX_POSITION] - position).Unit
                table.insert(blockedDirections, delta)
            elseif (myCost > neighbor[IDX_TOTALCOST]) then
                --Use previously-explored neighbors to decide the direction things should go from here
                local neighborDirection = neighbor[IDX_DIRECTION]
                local center = neighbor[IDX_POSITION] + neighborDirection
                numCenters += 1
                centerPoint += center
                table.insert(parentDirections, neighborDirection)
                table.insert(parentCenters, center)
            end
        end
        --If we have a center position our direction should point to that isn't zero, handle it
        if (numCenters > 0) and centerPoint ~= Vector3.zero then
            centerPoint /= numCenters
            local direction: Vector3 = centerPoint - position
            if #blockedDirections > 0 then
                local originalMagnitude = direction.Magnitude
                --For each neighbor that was blocked, do a vector rejection if cell direction is heading that way
                for _, rejection in blockedDirections do
                    local scalarProjection = direction:Dot(rejection) / rejection:Dot(rejection)
                    if scalarProjection > 0 then
                        direction -= scalarProjection * rejection
                    end
                end
                --Adjust the magnitude so it properly blends with other cell directions later on
                if direction ~= Vector3.zero then
                    direction = direction.Unit * originalMagnitude
                end
            end
            if #parentDirections == 1 then
                if (math.abs(direction:Dot(parentDirections[1])) < 10^-10) then
                    direction = direction.Unit * math.min(direction.Magnitude, GRID_COORD_SPACING * 1)
                end
            elseif #parentDirections == 2 then
                local originalMagnitude = direction.Magnitude
                if (math.abs(parentDirections[1]:Dot(parentDirections[2])) < 10^-10) and (parentDirections[1].Magnitude + parentDirections[2].Magnitude) > 0 then
                    local bestDist1 = math.huge
                    local bestDist2 = math.huge
                    for _, goalCell in goalCells do
                        local goalPos = goalCell[IDX_POSITION]
                        bestDist1 = math.min(bestDist1, (goalPos - parentCenters[1]).Magnitude)
                        bestDist2 = math.min(bestDist2, (goalPos - parentCenters[2]).Magnitude)
                    end
                    if bestDist1 < bestDist2 then
                        direction = (parentCenters[1] - position).Unit * originalMagnitude
                        if direction == Vector3.zero then
                            direction = parentDirections[1]
                        end
                    else
                        direction = (parentCenters[2] - position).Unit * originalMagnitude
                        if direction == Vector3.zero then
                            direction = parentDirections[2]
                        end
                    end
                end
            end
            currentCell[IDX_DIRECTION] = direction --.Unit * math.min(direction.Magnitude, GRID_COORD_SPACING * 33)
        end

        table.clear(blockedDirections)
        table.clear(parentDirections)
        table.clear(parentCenters)
    end

    --Normalize all the directions now that they're computed
    for index, cell in cells do
        local direction = cell[IDX_DIRECTION]
        if not direction:FuzzyEq(Vector3.zero) then
            cell[IDX_DIRECTION] = direction.Unit * GRID_COORD_SPACING
        end
        --if cell[IDX_BLOCKED] then
        --    cell[IDX_TOTALCOST] = math.huge
        --end
        if direction:FuzzyEq(Vector3.zero) and not field.BorderCells[index] and not table.find(goalCells, cell) then
            local blockCount = 0
            local neighbors = cell[IDX_NEIGHBORS]
            for _, neighbor in neighbors do
                blockCount += if neighbor[IDX_BLOCKED] then 1 else 0
            end
            if blockCount < #neighbors then
                warn("Danger! No direction! ", cell[IDX_POSITION], ";", cell)
            end
            direction = (goalCells[1][IDX_POSITION] - cell[IDX_POSITION]).Unit * GRID_COORD_SPACING
            cell[IDX_DIRECTION] = direction
        end
        if (direction.X ~= direction.X) or (direction.Z ~= direction.Z) then
            warn("Cell with NaN direction!", cell[IDX_POSITION], ";", cell)
        end
    end

    return true
end


--Update the costs so the field points the way to the goal
function FlowField.UpdateField(field: Field, goals: {Vector3}): boolean
    local goalCells: {FieldCell} = table.create(#goals)
    for i, goal in goals do
        local goalCellPos = ToCellGridRound(goal)
        local goalCell = field.Cells[CoordToIndex(goalCellPos.X, goalCellPos.Z)]

        if not ValidateCell("Goal", goal, goalCell) then
            return false
        end
        goalCells[i] = goalCell
    end

    return UpdateField(field, goalCells)
end


--[[
    Calculate whether a line between two points includes only unblocked cells.
    NOTE: The points need not be aligned to the field grid!
    Returns false iff the line crosses any grid coordinate with no cell or a cell marked blocked.

    Based on "A Fast Voxel Traversal Algorithm for Ray Tracing" by John Amanatides and Andrew Woo
    Extended to be a kind of "circle cast" against the field grid by tracing two lines.
--]]
local function IsLineTraversableUncached(field: Field, start: Vector3, finish: Vector3): boolean
    --Radius can't exceed (or maybe equal?) GRID_COORD_SPACING/2 or cells might be missed
    local radius = (GRID_COORD_SPACING / 2) - (2^-16)
    local cells = field.Cells
    local cell: FieldCell

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
    local offsetX = DeltaXCoordToIndex(stepX)
    local offsetZ = DeltaZCoordToIndex(stepZ)

    --Two points opposite a circle by the normal of the line
    local rduX = radius * du.X
    local rduZ = radius * du.Z
    local pAX = startX + rduZ --Yes, the X has the delta-unit Z added because it's perpendicular to the line
    local pAZ = startZ - rduX --And vice versa. Perpendicular slope to (X, Z) is (Z, -X)
    local pBX = startX - rduZ --And the other perpendicular slope to (X, Z) is (-Z, X)
    local pBZ = startZ + rduX
    --Those coordinates snapped to the grid
    local pAXGrid = ToCellGridRoundSingle(pAX)
    local pAZGrid = ToCellGridRoundSingle(pAZ)
    local pBXGrid = ToCellGridRoundSingle(pBX)
    local pBZGrid = ToCellGridRoundSingle(pBZ)

    local index = CoordToIndex(pAXGrid, pAZGrid)

    --If the line is horizontal or vertical, special processinng is required to avoid division by zero
    if deltaX == 0 then
        --For the loop count the Z endpoint is needed
        local pAEndZ = finish.Z - rduX
        local pAEndZGrid = ToCellGridRoundSingle(pAEndZ)

        --If the two lines representing the circle are in the same cell, only check one cell along the way
        if pAXGrid == pBXGrid then
            for zz = pAZGrid, pAEndZGrid, stepZ do
                cell = cells[index]
                if not cell or cell[IDX_BLOCKED] then
                    return false
                end
                index += offsetZ
            end
        else
            --Otherwise, check one cell for each line
            local indexDelta
            if pAXGrid < pBXGrid then
                indexDelta = DeltaXCoordToIndex(GRID_COORD_SPACING)
            else
                indexDelta = DeltaXCoordToIndex(-GRID_COORD_SPACING)
            end

            for zz = pAZGrid, pAEndZGrid, stepZ do
                cell = cells[index]
                if not cell or cell[IDX_BLOCKED] then
                    return false
                end
                cell = cells[index + indexDelta]
                if not cell or cell[IDX_BLOCKED] then

                    return false
                end
                index += offsetZ
            end
        end

        return true
    elseif deltaZ == 0 then
        --For the loop count the X endpoint is needed
        local pAEndX = finish.X + rduZ
        local pAEndXGrid = ToCellGridRoundSingle(pAEndX)

        --If the two lines representing the circle are in the same cell, only check one cell along the way
        if pAZGrid == pBZGrid then
            for xx = pAXGrid, pAEndXGrid, stepX do
                cell = cells[index]
                if not cell or cell[IDX_BLOCKED] then
                    return false
                end
                index += offsetX
            end
        else
            --Otherwise, check one cell for each line
            local indexDelta
            if pAZGrid < pBZGrid then
                indexDelta = DeltaZCoordToIndex(GRID_COORD_SPACING)
            else
                indexDelta = DeltaZCoordToIndex(-GRID_COORD_SPACING)
            end

            for xx = pAXGrid, pAEndXGrid, stepX do
                cell = cells[index]
                if not cell or cell[IDX_BLOCKED] then
                    return false
                end
                cell = cells[index + indexDelta]
                if not cell or cell[IDX_BLOCKED] then
                    return false
                end
                index += offsetX
            end
        end

        return true
    end

    --The T values at which a grid crossing takes place
    local halfStepX = stepX * 0.5
    local halfStepZ = stepZ * 0.5
    local tMaxXA = (pAXGrid + halfStepX - pAX) / deltaX
    local tMaxZA = (pAZGrid + halfStepZ - pAZ) / deltaZ
    local tMaxXB = (pBXGrid + halfStepX - pBX) / deltaX
    local tMaxZB = (pBZGrid + halfStepZ - pBZ) / deltaZ

    --The change in T a step in each direction adavances.
    local tDeltaX = stepX / deltaX
    local tDeltaZ = stepZ / deltaZ

    --Step line A between X and Z grid crossings, checking each cell along the field
    while (tMaxXA < 1) or (tMaxZA < 1) do
        cell = cells[index]
        if not cell or cell[IDX_BLOCKED] then
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

    --Check the final cell that line A hits
    cell = cells[index]
    if not cell or cell[IDX_BLOCKED] then
        return false
    end

    --Step line B between X and Z grid crossings, checking each cell along the field
    index = CoordToIndex(pBXGrid, pBZGrid)
    while (tMaxXB < 1) or (tMaxZB < 1) do
        cell = cells[index]
        if not cell or cell[IDX_BLOCKED] then
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

    --Check the final cell that line B hits
    cell = cells[index]
    if not cell or cell[IDX_BLOCKED] then
        return false
    end

    return true
end
FlowField.IsLineTraversableUncached = IsLineTraversableUncached


--Wrapper of IsLineTraversable that uses a cache for grid-aligned queries
local function IsLineTraversableCached(field: Field, start: Vector3, finish: Vector3): boolean
    if not IsOnPathGrid(start) or not IsOnPathGrid(finish) then
        return IsLineTraversableUncached(field, start, finish)
    end

    local startIndex = CoordToIndex(start.X, start.Z)
    local finishIndex = CoordToIndex(finish.X, finish.Z)

    local lineCache = field.LineCache
    local index = CombineIndexesIntoLine(startIndex, finishIndex)

    local value = lineCache[index]
    if value ~= nil then
        return value
    end

    value = IsLineTraversableUncached(field, start, finish)
    lineCache[index] = value
    return value
end
FlowField.IsLineTraversable = IsLineTraversableCached


--Calculate whether a rectangle between two points includes only unblocked cells.
function FlowField.IsAreaUnblocked(field: Field, minCorner: Vector3, maxCorner: Vector3, includePartial: boolean?): boolean
    local cells = field.Cells

    minCorner, maxCorner = AlignAreaToGrid(minCorner, maxCorner, includePartial)
    local minCornerX = minCorner.X
    local maxCornerX = maxCorner.X

    for z = minCorner.Z, maxCorner.Z, GRID_COORD_SPACING do
        local index = CoordToIndex(minCornerX, z)

        for x = minCornerX, maxCornerX, GRID_COORD_SPACING do
            local cell = cells[index]

            if not cell or cell[IDX_BLOCKED] then
                return false
            end

            index += DeltaXCoordToIndex(GRID_COORD_SPACING)
        end
    end

    return true
end


--Calculate whether a point is reachable from any goal point used to calculate costs
function FlowField.IsPointReachable(field: Field, point: Vector3): boolean
    local cellPos = ToCellGridRound(point)
    local cell = field.Cells[CoordToIndex(cellPos.X, cellPos.Z)]

    return cell and not cell[IDX_BLOCKED] and (cell[IDX_TOTALCOST] ~= math.huge)
end


--Calculate whether a point has an obstacle covering it
function FlowField.IsPointBlocked(field: Field, point: Vector3): boolean
    local cellPos = ToCellGridRound(point)
    local cell = field.Cells[CoordToIndex(cellPos.X, cellPos.Z)]

    return cell and cell[IDX_BLOCKED]
end


return table.freeze(FlowField)