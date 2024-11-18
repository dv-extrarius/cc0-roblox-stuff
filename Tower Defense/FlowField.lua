--[[
Copyright Extrarius 2023, 2024
This work is marked with CC0 1.0 Universal.
To view a copy of this license, visit http://creativecommons.org/publicdomain/zero/1.0
--
This is an implementation of an essentially 2d pathfinding system. It ignores Y coordinates, and
uses a grid of cells that are manually marked blocked or unblocked by the library's user.

This version uses flow fields, where each cell has a direction attribute that units can follow to the goal.
For smoother paths, the direction for a point is computed using bilinear interpolation from the four surrounding cells.

As far as I know, the way the directions are computed is novel. To get directions beyond the base 8 possible
from the 4-connected cells, an "origin" is tracked that a cell points towards, making new origins as the line
to previous origins is blocked by blocked cells.

Initial origins are created at each goal cell and the goal cell points to it.
A breadth-first wave expansion is used to calculate approximate distance and which cells are parents of which.
    Important: if a cell has 2 parents with identical costs not on opposite sides, the cost uses a diagonal step instead of 2 steps.
    This is important because it ensures the origin-finding loop will take shorter paths around obstacles
    (4-direction distance is too far off euclidian distance, but 8-direction distance is not)

A second pass is done over all cells (except goals - they have an origin already) in a sort of topological-distance order:
Each blocked cell gets a direction "out" of the blocked area so a unit that gets blocked will find a short way out.
Each passable cell takes the reachable origin from its parents that results in the lowest-cost path to a goal cell.
    Reachability is calculated by keeping track of which angles from each origin have met a blocked cell, as well as
        directly testing whether the direction to an origin crosses a blocked neighbor.
    Due to the order cells are visited, blocked angles are updated for all current parents' origins after each wave.
    If the path to all parents' origins is blocked, a new origin is created over a parent with the least cost.
]]
--!strict
--!optimize 2

local FlowField = {}

--Constants used to index the FieldCell type
local IDX_CELL_NEIGHBORS = 1
local IDX_CELL_POSITION = 2
local IDX_CELL_INDEX = 3
local IDX_CELL_TOTALCOST = 4
local IDX_CELL_BLOCKED = 5
local IDX_CELL_DIRECTION = 6
local IDX_CELL_WAVENUMBER = 7
local IDX_CELL_ORIGIN = 8
local IDX_CELLBORDER_IDSTRING = 9 --This index MUST NOT EXIST in normal cells

type FieldCell = {any} --Unfortuantely heterogeneous arrays not yet supported in type system
--[[
type FieldCell = {
    Neighbors: {FieldCell},
    Position: Vector3,
    Index: number,
    TotalCost: number,
    Blocked: boolean,
    Direction: Vector3,
    WaveNumber: number,
    Origin: CellOrigin,
}
]]
type AngleRange = Vector3 --X is start and Y is end (counterclockwise)

local IDX_ORIGIN_POSITION = 1
local IDX_ORIGIN_COST_TO_GOAL = 2
local IDX_ORIGIN_BLOCKED_ANGLES = 3
--local IDX_ORIGIN_BLOCKED_POSITIONS = 4
type CellOrigin = {any} --Unfortuantely heterogeneous arrays not yet supported in type system
--[[
type CellOrigin = {
    Position: Vector3,
    CostToGoal: number,
    BlockedAngles: {AngleRange},
--    BlockedPositions: {[number]: true}, --number is the coord index
}
]]

type FieldUpdateCache = {
    FrontWaveTable: {[FieldCell]: true},
    NextWaveTable: {[FieldCell]: true},
    FutureWaveTable: {[FieldCell]: true},
}

export type Field = {
    Cells: {[number]: FieldCell},
    SortedCells: {FieldCell},
    BorderCells: {[number]: FieldCell},
    IsFinalized: boolean,
    GoalCells: {FieldCell},
    UpdateCache: FieldUpdateCache,

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
they are value types, a benchmark showed my system is about 5% faster for my case, so it remains.
Since places won't be huge and cells are limited to integer coordinates, we can pack the coordinates
using some bits for each one. Since there are effectively 53 mantissa bits, I chose floor(53/3) bits
per coordinate (in case Y is used someday), resulting in a multiplier of 2^17 = 131072.
Thus, pathfinding cells can span the range of X and Z coordinates -65536 to +65535, or approximately far enough
]]
local GRID_INDEX_MUL = 131072


--Constants used for updating the field
local TAU = 2 * math.pi
local EXTRA_BLOCKED_CELL_COST = 2^24 --arbitrary large value much larger than the number of cells
local INVALID_ORIGIN: CellOrigin = table.freeze({
    Vector3.zero,                                      --Position
    math.huge,                                         --CostToGoal
    table.freeze({Vector3.new(-math.pi, math.pi, 0)}), --BlockedAngles
    --    table.freeze({}),                                  --BlockedPositions
} :: CellOrigin)


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
local COORD_INDEX_BASE = ((GRID_INDEX_MUL / 2) + (GRID_INDEX_MUL / 2) * GRID_INDEX_MUL)
local function CoordToIndex(x: number, z: number): number
    --TODO: Maybe take advantage of grid size to increase range?
    --Reorganizing terms results in better bytecode:
    --return (x + (GRID_INDEX_MUL / 2)) + ((z + (GRID_INDEX_MUL / 2)) * GRID_INDEX_MUL)
    return x + (z * GRID_INDEX_MUL) + COORD_INDEX_BASE
end
FlowField.CoordToIndex = CoordToIndex

--Convert an X coordinate to a cell index (with Z=0) (to be adjusted using Delta* functions)
local function XCoordToIndex(x: number): number
    --TODO: Maybe take advantage of grid size to increase range?
    return x + COORD_INDEX_BASE
end

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
        EnableDebugMessages = config.EnableDebugMessages and true
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
        UpdateCache = {
            FrontWaveTable = {},
            NextWaveTable = {},
            FutureWaveTable = {},
        },
    }
end


--Clone a field. Finalizes the cells if the original is finialized
function FlowField.Clone(oldField: Field, skipFinalize: boolean?): Field
    local newField: Field = {
        Cells = {},
        SortedCells = table.create(#oldField.SortedCells) :: {FieldCell},
        BorderCells = {},
        IsFinalized = false,
        GoalCells = table.create(#oldField.GoalCells) :: {FieldCell},
        UpdateCache = {
            FrontWaveTable = {},
            NextWaveTable = {},
            FutureWaveTable = {},
        },
    }
    local oldCells = oldField.Cells
    local oldSorted = oldField.SortedCells
    local newCells = newField.Cells
    local newSorted = newField.SortedCells

    for index, oldCell in oldCells do
        local newCell = table.clone(oldCell)

        newCell[IDX_CELL_NEIGHBORS] = table.create(4)
        newCells[index] = newCell
    end
    for _, oldGoal in oldField.GoalCells do
        table.insert(newField.GoalCells, newCells[oldGoal[IDX_CELL_INDEX]])
    end
    --Copy sorted cells from sorted cells, so the new one is sorted if the old one is
    for _, oldCell in oldSorted do
        table.insert(newSorted, newCells[oldCell[IDX_CELL_INDEX]])
    end

    if oldField.IsFinalized and not skipFinalize then
        local newBorderCells = oldField.BorderCells
        newField.BorderCells = newBorderCells
        --Connect each cell to its 4-way neighbors
        for cellIndex, newCell in newCells do
            local newNeighbors: {FieldCell} = newCell[IDX_CELL_NEIGHBORS]
            local oldNeighbors: {FieldCell} = oldCells[cellIndex][IDX_CELL_NEIGHBORS]

            for i, neighbor in oldNeighbors do
                local neighborIndex = neighbor[IDX_CELL_INDEX]
                newNeighbors[i] = newCells[neighborIndex] or newBorderCells[neighborIndex]
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
                    table.create(4),       --Neighbors
                    Vector3.new(x, 0, z),  --Position
                    index,                 --Index
                    math.huge,             --TotalCost
                    false,                 --Blocked
                    Vector3.zero,          --Direction
                    math.huge,             --WaveNumber
                    INVALID_ORIGIN,        --Origin
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
    return {
        frozenEmptyNeighbors, --Neighbors
        borderPosition,       --Position
        neighborIndex,        --Index
        math.huge,            --TotalCost
        true,                 --Blocked
        Vector3.zero,         --Direction
        math.huge,            --WaveNumber
        INVALID_ORIGIN,       --Origin
        borderCellStr         --IDX_CELLBORDER_IDSTRING
    } :: FieldCell
end


--Do final processing on each cell in the field
function FlowField.FinalizeField(field: Field): ()
    local cells = field.Cells
    local borderCells = field.BorderCells
    local sorted = field.SortedCells
    local neighborIndex: number

    --Make the sorted cells actually sorted (by index)
    table.sort(sorted, function (lhs, rhs) return lhs[IDX_CELL_INDEX] < rhs[IDX_CELL_INDEX] end)

    --Connect each cell to 4-way neighbors if they exist and create border cells if they don't
    for index, currentCell in cells do
        local neighbors: {FieldCell} = currentCell[IDX_CELL_NEIGHBORS]
        local neighbor: FieldCell?

        neighborIndex = index + DeltaXCoordToIndex(-GRID_COORD_SPACING)
        neighbor = cells[neighborIndex] or borderCells[neighborIndex]
        if not neighbor then
            neighbor = MakeBorderCell(neighborIndex, currentCell[IDX_CELL_POSITION] - GRID_COORD_SPACING*Vector3.xAxis)
            borderCells[neighborIndex] = neighbor
        end
        table.insert(neighbors, neighbor :: FieldCell)


        neighborIndex = index + DeltaXCoordToIndex(GRID_COORD_SPACING)
        neighbor = cells[neighborIndex] or borderCells[neighborIndex]
        if not neighbor then
            neighbor = MakeBorderCell(neighborIndex, currentCell[IDX_CELL_POSITION] + GRID_COORD_SPACING*Vector3.xAxis)
            borderCells[neighborIndex] = neighbor
        end
        table.insert(neighbors, neighbor :: FieldCell)


        neighborIndex = index + DeltaZCoordToIndex(-GRID_COORD_SPACING)
        neighbor = cells[neighborIndex] or borderCells[neighborIndex]
        if not neighbor then
            neighbor = MakeBorderCell(neighborIndex, currentCell[IDX_CELL_POSITION] - GRID_COORD_SPACING*Vector3.zAxis)
            borderCells[neighborIndex] = neighbor
        end
        table.insert(neighbors, neighbor :: FieldCell)


        neighborIndex = index + DeltaZCoordToIndex(GRID_COORD_SPACING)
        neighbor = cells[neighborIndex] or borderCells[neighborIndex]
        if not neighbor then
            neighbor = MakeBorderCell(neighborIndex, currentCell[IDX_CELL_POSITION] + GRID_COORD_SPACING*Vector3.zAxis)
            borderCells[neighborIndex] = neighbor
        end
        table.insert(neighbors, neighbor :: FieldCell)


        table.freeze(neighbors)
    end

    --Set direction for border cells to be away from all non-border neighbors
    for index, borderCell in borderCells do
        local myPosition: Vector3 = borderCell[IDX_CELL_POSITION]
        local direction: Vector3 = Vector3.zero
        local neighbor: FieldCell?

        neighborIndex = index + DeltaXCoordToIndex(-GRID_COORD_SPACING)
        neighbor = cells[neighborIndex]
        if neighbor then
            direction += ((neighbor[IDX_CELL_POSITION] :: Vector3) - myPosition)
        end

        neighborIndex = index + DeltaXCoordToIndex(GRID_COORD_SPACING)
        neighbor = cells[neighborIndex]
        if neighbor then
            direction += ((neighbor[IDX_CELL_POSITION] :: Vector3) - myPosition)
        end

        neighborIndex = index + DeltaZCoordToIndex(-GRID_COORD_SPACING)
        neighbor = cells[neighborIndex]
        if neighbor then
            direction += ((neighbor[IDX_CELL_POSITION] :: Vector3) - myPosition)
        end

        neighborIndex = index + DeltaZCoordToIndex(GRID_COORD_SPACING)
        neighbor = cells[neighborIndex]
        if neighbor then
            direction += ((neighbor[IDX_CELL_POSITION] :: Vector3) - myPosition)
        end

        borderCell[IDX_CELL_DIRECTION] = direction.Unit * GRID_COORD_SPACING
        table.freeze(borderCell)
    end

    field.IsFinalized = true
    table.freeze(cells)
    table.freeze(borderCells)
    table.freeze(sorted)
end


--Clear caches that are invalidated when blocked status changees
local function ResetCachesForBlockChange(field: Field)
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
                cell[IDX_CELL_BLOCKED] = true
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
                cell[IDX_CELL_BLOCKED] = false
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
        if cell[IDX_CELL_BLOCKED] then
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
            cell[IDX_CELL_BLOCKED] = true
        else
            cell[IDX_CELL_BLOCKED] = false
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
    elseif cell[IDX_CELL_BLOCKED] then
        CustomWarn(label, " cell at (", location, ") is blocked!")
        return false
    end
    return true
end


--Utility to simplify calling .Unit on vectors that may be zero
local function SafeUnit(vec: Vector3) :Vector3
    if vec ~= Vector3.zero then
        return vec.Unit
    else
        return vec
    end
end

local angleRangesCache: {AngleRange} = (table.create(1) :: any)
--Create a new angle range from an existin range and a new range. Steals the input list for cache use
local function AddAngleToAngleRange(ranges: {AngleRange}, addRange: AngleRange): {AngleRange}
    --If the angle straddles the wrap from +180 to -180, we have to break it up into two angle ranges
    if addRange.X > addRange.Y then
        ranges = AddAngleToAngleRange(ranges, Vector3.new(addRange.X, math.pi, 0))
        addRange = Vector3.new(-math.pi, addRange.Y, 0)
    end

    local numRanges: number = #ranges
    if numRanges == 0 then
        --(#1 most common case) If this is the first range inserted, just set it
        ranges[1] = addRange
        return ranges

    elseif numRanges == 1 then
        --(#2 most common case)  Merge the new angles into a range with exactly 1 entry

        local addStart = addRange.X
        local addEnd = addRange.Y

        local rangeStart = ranges[1].X
        local rangeEnd = ranges[1].Y

        if rangeEnd < addStart then
            ranges[2] = addRange
        elseif addEnd < rangeStart then
            table.insert(ranges, 1, addRange)
        else
            ranges[1] = Vector3.new(math.min(addStart, rangeStart), math.max(addEnd, rangeEnd), 0)
        end
        return ranges

    else
        --Handle the least common case of merging into a range with 2+ angles already
        local addStart = addRange.X
        local addEnd = addRange.Y

        local newRanges = angleRangesCache
        local placed = false

        for _, range in ranges do
            local rangeStart = range.X
            local rangeEnd = range.Y
            if rangeEnd < addStart then
                --no overlap
                table.insert(newRanges, range)
            elseif addEnd < rangeStart then
                --no overlap, insert new range before this one
                if not placed then
                    table.insert(newRanges, Vector3.new(addStart, addEnd))
                    placed = true
                end
                table.insert(newRanges, range)
            else
                --overlap, update variables to include new range and continue
                addStart = math.min(addStart, rangeStart)
                addEnd = math.max(addEnd, rangeEnd)
            end
        end
        if not placed then
            table.insert(newRanges, Vector3.new(addStart, addEnd))
        end

        angleRangesCache = ranges
        table.clear(angleRangesCache)

        return newRanges
    end
end

--Check whether an angle range includes a given angle
local function DoesAngleRangeContainAngle(ranges: {AngleRange}, angle: number): boolean
    --Origins usually only have 0 or 1 angle ranges, so linear search is faster than binary
    for _, range in ranges do
        if angle >= range.X then
            if angle <= range.Y then
                return true
            end
        else
            return false
        end
    end
    return false
end


--Calculate the angle range occluded by a square from a given origin, expanded to account for "unit" size equal to grid size
local function SquareAnglesExpanded(origin: Vector3, squareCenter: Vector3): (number, number)
    local halfGridSpacing = 0.5 * GRID_COORD_SPACING

    local delta = squareCenter - origin

    local xCenter = delta.X
    local yCenter = delta.Z
    --Center adjusted to be in quadruant 1
    local absXCenter = math.abs(xCenter)
    local absYCenter = math.abs(yCenter)
    --Perpendicular offsets to account for object radius
    local du = delta.Unit
    local perpX = math.abs(du.Z) * halfGridSpacing
    local perpY = -math.abs(du.X) * halfGridSpacing

    --Corners Offsets from Center: (1) -X,+Y;  (2) -X,-Y; (3) +X,-Y
    --Use cornerA = 3 or 2 depending on whether the square crosses Y axis
    local xCornerA: number = absXCenter
    local yCornerA: number = absYCenter - halfGridSpacing + perpY
    if (yCornerA < 0) then
        --Use cornerA = 2 when crossig Y axis
        xCornerA -= halfGridSpacing
    else
        --Use cornerA = 3 otherwise
        xCornerA += halfGridSpacing
    end
    xCornerA += perpX

    --Use CornerB = 1 or 2 depending on whether the square crosses X axis
    local xCornerB: number = absXCenter - halfGridSpacing - perpX
    local yCornerB: number = absYCenter
    if (xCornerB < 0) then
        --Use cornerB = 2 when crossing X axis
        yCornerB -= halfGridSpacing
    else
        --Use cornerB = 1 otherwise
        yCornerB += halfGridSpacing
    end
    yCornerB -= perpY

    local thetaA = math.atan2(yCornerA, xCornerA)
    local thetaB = math.atan2(yCornerB, xCornerB)

    --Adjust for original quadruant
    if xCenter < 0 then
        --Originally quadruant 2 or 4
        thetaB, thetaA = (math.pi - thetaA), (math.pi - thetaB)
    end
    if yCenter < 0 then
        --Originally quadrant 3 or 4
        thetaB, thetaA = -thetaA, -thetaB
    end
    --Wrap to range -pi..pi
    thetaB = (thetaB + math.pi) % TAU - math.pi
    thetaA = (thetaA + math.pi) % TAU - math.pi

    return thetaA, thetaB
end


local function MakeOrigin(position: Vector3, costToGoal: number): CellOrigin
    local origin: CellOrigin = {
        position,        --Position
        costToGoal,      --CostToGoal
        table.create(1), --BlockedAngles
        --        {},              --BlockedPositions
    }
    return origin
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
        cell[IDX_CELL_TOTALCOST] = math.huge
        cell[IDX_CELL_DIRECTION] = Vector3.zero
        cell[IDX_CELL_ORIGIN] = INVALID_ORIGIN
        cell[IDX_CELL_WAVENUMBER] = math.huge
    end

    --No need to clone the goal table, only place it comes from is where we create it
    field.GoalCells = goalCells

    --Do a full expansion of cells in waves until all unblocked cells have been assigned the minimum cost to get there
    local frontWave = field.UpdateCache.FrontWaveTable
    local nextWave = field.UpdateCache.NextWaveTable
    local futureWave = field.UpdateCache.FutureWaveTable

    table.clear(frontWave)
    table.clear(nextWave)
    table.clear(futureWave)

    --Info used to update angle ranges for origins
    local originUpdates: {[CellOrigin]: {[number]: Vector3}} = {}

    --Initialize goal cells to 0 distance and set the first wave you contain them
    for _, goalCell in goalCells do
        frontWave[goalCell] = true
        goalCell[IDX_CELL_TOTALCOST] = 0
        local origin = MakeOrigin(goalCell[IDX_CELL_POSITION], 0)
        goalCell[IDX_CELL_ORIGIN] = origin
        originUpdates[origin] = {}
    end

    --debug.profilebegin("WaveExpansion")
    local currentWaveNumber = 0
    repeat
        --Exhaust the current wave to update cost of all reachable areas
        while next(frontWave) do
            currentWaveNumber += 1
            for currentCell, _ in frontWave do
                local myCost = currentCell[IDX_CELL_TOTALCOST]
                local totalCost = myCost + 1
                local blockedTotalCost = totalCost + EXTRA_BLOCKED_CELL_COST
                local myIsBlocked: boolean = currentCell[IDX_CELL_BLOCKED]
                local myPosition = currentCell[IDX_CELL_POSITION]

                currentCell[IDX_CELL_WAVENUMBER] = currentWaveNumber

                --Update this cell's cost to account for diagonal movement if appropriate
                if not myIsBlocked then
                    --Collect the location and cost of all parent cells with equal cost
                    local minParentCost = math.huge
                    local numMatchingParents = 0
                    local averageParentCoord = Vector3.zero
                    for _, neighbor in currentCell[IDX_CELL_NEIGHBORS] do
                        local neighborCost = neighbor[IDX_CELL_TOTALCOST]
                        if neighborCost < minParentCost then
                            minParentCost = neighborCost
                            numMatchingParents = 1
                            averageParentCoord = neighbor[IDX_CELL_POSITION]
                        elseif neighborCost == minParentCost then
                            numMatchingParents += 1
                            averageParentCoord += neighbor[IDX_CELL_POSITION]
                        end
                    end
                    --If the parents are acually parents and we have exactly 2
                    if (minParentCost < myCost) and (numMatchingParents == 2) then
                        averageParentCoord /= numMatchingParents
                        --And the two parents are not on opposite sides
                        if not averageParentCoord:FuzzyEq(myPosition, 0.00001) then
                            --Adjust cost as if diagonal movement were allowed
                            myCost += (1.4143 - 2)
                            totalCost = myCost + 1
                            currentCell[IDX_CELL_TOTALCOST] = myCost
                        end
                    end
                end

                --Process each neighbor to see which cells should be expanded from here
                for _, neighbor in currentCell[IDX_CELL_NEIGHBORS] do
                    local neighborIsBlocked = neighbor[IDX_CELL_BLOCKED]

                    if neighborIsBlocked then
                        --Blocked neighbors get extra cost to ensure they're not desirable paths
                        local neighborIndex = neighbor[IDX_CELL_INDEX]
                        if (blockedTotalCost < neighbor[IDX_CELL_TOTALCOST]) and not neighbor[IDX_CELLBORDER_IDSTRING] then
                            --Explore blocked neighbors in a future wave, since their neighbors are probably neighbors soon-visited unblocked nodes
                            neighbor[IDX_CELL_TOTALCOST] = blockedTotalCost
                            futureWave[neighbor] = true
                        end
                    elseif (totalCost < neighbor[IDX_CELL_TOTALCOST]) then
                        neighbor[IDX_CELL_TOTALCOST] = totalCost
                        if myIsBlocked then
                            --Don't allow expanding from blocked->unblocked until the far future
                            futureWave[neighbor] = true
                        else
                            --Neighbors whose costs can be lowered can be explored next wave
                            nextWave[neighbor] = true
                            futureWave[neighbor] = nil
                        end
                    end
                end
            end
            --Advance to the next wave, which is updated neighbors of the current wave
            frontWave, nextWave = nextWave, frontWave
            table.clear(nextWave)
        end
        --Advance to the future wave, including to previously-blocked areas
        frontWave, futureWave = futureWave, frontWave
        table.clear(futureWave)
    until not next(frontWave)

    --debug.profileend()

    --Bin cells into waves based on when they were explored to ensure parent cells are updated before their children
    local sortedWaves = table.create(currentWaveNumber)
    for i = 1, currentWaveNumber do
        sortedWaves[i] = {}
    end
    for index, cell in cells do
        local waveNumber = cell[IDX_CELL_WAVENUMBER]
        if waveNumber ~= math.huge then
            table.insert(sortedWaves[waveNumber], cell)
        end
    end

    --Goals already have origins, so don't reprocess them
    table.clear(sortedWaves[1])

    --Several pieces of information about neighbors are needed to compute directions for each non-blocked cell
    local blockedDirections: {Vector3} = table.create(4)
    local parentOrigins: {CellOrigin} = table.create(4)
    local parents: {FieldCell} = table.create(4)
    local blockedPositions: {[number]: Vector3} = {}--unfortunately no table.create equivalent for dictionaries

    --Several pieces of information about neighbors are needed to compute directions for each blocked cell
    local unblockedParentDirections: {Vector3} = table.create(4)
    local blockedParentDirections: {Vector3} = table.create(4)

    --debug.profilebegin("AssignOrigins")
    --Process all the nodes in the waves they were
    for myWave, currentWave in sortedWaves do
        if #currentWave > 1 then
            table.sort(currentWave, function (a, b) return a[IDX_CELL_TOTALCOST] < b[IDX_CELL_TOTALCOST] end)
        end

        for _, currentCell in currentWave do
            local myCost = currentCell[IDX_CELL_TOTALCOST]
            local myPosition: Vector3 = currentCell[IDX_CELL_POSITION]
            local myIsBlocked: boolean = currentCell[IDX_CELL_BLOCKED]


            if myIsBlocked then
                --debug.profilebegin("BlockedCell")
                --Blocked cells just need to point to nearby unblocked cells
                table.clear(unblockedParentDirections)
                table.clear(blockedParentDirections)
                table.clear(parents)

                local nearestNeighborWave = math.huge
                for _, neighbor in currentCell[IDX_CELL_NEIGHBORS] do
                    local neighborWave = neighbor[IDX_CELL_WAVENUMBER]
                    nearestNeighborWave = math.min(nearestNeighborWave, neighborWave)

                    --Blocked cells get expanded almost simultaneously because of wave order, so only count previous waves as parents
                    local neighborIsParent = (myWave > neighborWave)
                    if neighborIsParent then
                        local neighborIsBlocked = neighbor[IDX_CELL_BLOCKED]

                        table.insert(parents, neighbor)
                        if neighborIsBlocked then
                            table.insert(blockedParentDirections, neighbor[IDX_CELL_DIRECTION])
                        else
                            local neighborPosition = neighbor[IDX_CELL_POSITION]
                            local delta = SafeUnit(neighborPosition - myPosition)
                            table.insert(unblockedParentDirections, delta)
                        end
                    end
                end

                --If this cell doesn't have parents yet, reschedule it for a later wave
                if #parents == 0 then
                    local newWaveNumber = nearestNeighborWave + 1
                    currentCell[IDX_CELL_WAVENUMBER] = newWaveNumber
                    if newWaveNumber > currentWaveNumber then
                        currentWaveNumber = newWaveNumber
                        if newWaveNumber > #sortedWaves then
                            sortedWaves[newWaveNumber] = {}
                        end
                    end
                    table.insert(sortedWaves[newWaveNumber], currentCell)
                    --debug.profileend()
                    continue
                end

                --Calculate a direction vector based on the parents
                local meanDirection = Vector3.zero
                if #unblockedParentDirections > 0 then
                    for _, direction in unblockedParentDirections do
                        meanDirection += direction
                    end
                    meanDirection /= #unblockedParentDirections
                elseif #blockedParentDirections > 0 then
                    for _, direction in blockedParentDirections do
                        meanDirection += direction
                    end
                    meanDirection /= #blockedParentDirections
                end
                --If the parent directions cancel eachother, just point to parent with lowest cost
                if meanDirection == Vector3.zero then
                    local bestParentCost = math.huge
                    for _, parent in parents do
                        local parentCost = parent[IDX_CELL_TOTALCOST]
                        if parentCost < bestParentCost then
                            bestParentCost = parentCost
                            meanDirection = parent[IDX_CELL_POSITION] - myPosition
                        end
                    end
                end

                currentCell[IDX_CELL_DIRECTION] = 4 * GRID_COORD_SPACING * SafeUnit(meanDirection)
                --debug.profileend()
            else ----------------------------------------------------------------------------------------------------------------------------------

                --debug.profilebegin("PassableCell")

                table.clear(blockedDirections)
                table.clear(parentOrigins)
                table.clear(parents)
                table.clear(blockedPositions)

                --debug.profilebegin("ScanNeighbors")
                local nearestNeighborWave = math.huge
                for _, neighbor in currentCell[IDX_CELL_NEIGHBORS] do
                    local neighborWave = neighbor[IDX_CELL_WAVENUMBER]
                    nearestNeighborWave = math.min(nearestNeighborWave, neighborWave)

                    local neighborIsParent = (myWave > neighborWave) or ((myWave == neighborWave) and (myCost > neighbor[IDX_CELL_TOTALCOST]))

                    if neighborIsParent then
                        local neighborOrigin = neighbor[IDX_CELL_ORIGIN]
                        table.insert(parents, neighbor)
                        if neighborOrigin then
                            table.insert(parentOrigins, neighborOrigin)
                        end
                    elseif neighbor[IDX_CELL_BLOCKED] then
                        local neighborPosition = neighbor[IDX_CELL_POSITION]
                        local neighborIndex = neighbor[IDX_CELL_INDEX]
                        local delta = SafeUnit(neighborPosition - myPosition)
                        --Record the direction to non-parent blocking cells to avoid going that way
                        table.insert(blockedDirections, delta)
                        blockedPositions[neighborIndex] = neighborPosition
                    end
                end
                --debug.profileend()

                --If this cell doesn't have parents yet, reschedule it for a later wave
                if #parents == 0 then
                    local newWaveNumber = nearestNeighborWave + 1
                    currentCell[IDX_CELL_WAVENUMBER] = newWaveNumber
                    if newWaveNumber > currentWaveNumber then
                        currentWaveNumber = newWaveNumber
                        if newWaveNumber > #sortedWaves then
                            sortedWaves[newWaveNumber] = {}
                        end
                    end
                    table.insert(sortedWaves[newWaveNumber], currentCell)
                    --debug.profileend()
                    continue
                end

                --Examine all the origins of parent cells to find the best one for this cell
                --debug.profilebegin("Origins")
                local bestOrigin: CellOrigin = nil
                local bestOriginCost = math.huge
                for _, origin in parentOrigins do
                    if origin == INVALID_ORIGIN then
                        continue
                    end
                    if next(blockedPositions) then
                        --debug.profilebegin("ScheduleUpdates")
                        --Insert future updates for all blocked neighbors
                        local updates = originUpdates[origin]
                        if not updates then
                            updates = {}
                            originUpdates[origin] = updates
                        end
                        for blockedIndex, blockedPosition in blockedPositions do
                            updates[blockedIndex] = blockedPosition
                        end
                        --debug.profileend()
                    end
                    --debug.profilebegin("OriginTests")
                    --Check if this origin can be the best one and is valid for this cell
                    local deltaFromOrigin = myPosition - origin[IDX_ORIGIN_POSITION]
                    local originCost = origin[IDX_ORIGIN_COST_TO_GOAL] + deltaFromOrigin.Magnitude
                    if originCost >= bestOriginCost then
                        --debug.profileend()
                        continue
                    elseif DoesAngleRangeContainAngle(origin[IDX_ORIGIN_BLOCKED_ANGLES], math.atan2(deltaFromOrigin.Z, deltaFromOrigin.X)) then
                        --debug.profileend()
                        continue
                    end
                    --debug.profileend()
                    --debug.profilebegin("OriginValidate")
                    local deltaFromCell = -deltaFromOrigin
                    local isOriginBlocked = false
                    for _, blockedDirection in blockedDirections do
                        --Check if the direction of the scalar projections indicates we go towards a blocked direction
                        --Since we're just checking the sign, the exact magnitude doesn't matter and we can skip the division by a positive number
                        local scalarProjection = deltaFromCell:Dot(blockedDirection)-- / blockedDirection:Dot(blockedDirection)
                        if scalarProjection > 0 then
                            isOriginBlocked = true
                            break
                        end
                    end
                    if not isOriginBlocked then
                        bestOrigin = origin
                        bestOriginCost = originCost
                    end
                    --debug.profileend()
                end
                --debug.profileend()

                --If no parent cells are good candidates, make a new origin for this cell in one of its parents
                if not bestOrigin then
                    --debug.profilebegin("MakeNewOrigin")
                    local bestOriginPosition = myPosition
                    for _, parent in parents do
                        local parentPosition: Vector3 = parent[IDX_CELL_POSITION]
                        local parentOrigin: CellOrigin = parent[IDX_CELL_ORIGIN]
                        local parentCost: number
                        if not parent[IDX_CELL_BLOCKED] then
                            parentCost = (parentPosition - parentOrigin[IDX_ORIGIN_POSITION]).Magnitude + parentOrigin[IDX_ORIGIN_COST_TO_GOAL]
                        else
                            parentCost = parent[IDX_CELL_TOTALCOST]
                        end
                        if parentCost < bestOriginCost then
                            bestOriginPosition = parentPosition
                            bestOriginCost = parentCost
                        end
                    end
                    bestOrigin = MakeOrigin(bestOriginPosition, bestOriginCost)

                    --Ensure a new origin gets proper updates
                    local updates = {}
                    originUpdates[(bestOrigin :: any) :: CellOrigin] = updates
                    for blockedIndex, blockedPosition in blockedPositions do
                        updates[blockedIndex] = blockedPosition
                    end
                    --debug.profileend()
                end

                currentCell[IDX_CELL_ORIGIN] = bestOrigin
                currentCell[IDX_CELL_DIRECTION] = GRID_COORD_SPACING * SafeUnit(bestOrigin[IDX_ORIGIN_POSITION] - myPosition)
                --debug.profileend()
            end
        end

        --debug.profilebegin("UpdateOrigins")
        --Apply all updates to origins
        for origin, blockedCellPositions in originUpdates do
            --local updated = false
            local originPosition = origin[IDX_ORIGIN_POSITION]
            --local blockedOriginPositions = origin[IDX_ORIGIN_BLOCKED_POSITIONS]
            local originBlockedAngles = origin[IDX_ORIGIN_BLOCKED_ANGLES]
            for blockedIndex, blockedCellPosition in blockedCellPositions do
                --if not blockedOriginPositions[blockedIndex] then
                --    blockedOriginPositions[blockedIndex] = true
                local startAngle, endAngle = SquareAnglesExpanded(originPosition, blockedCellPosition)
                originBlockedAngles = AddAngleToAngleRange(originBlockedAngles, Vector3.new(startAngle, endAngle, 0))
                --    updated = true
                --end
            end
            --if updated then
            origin[IDX_ORIGIN_BLOCKED_ANGLES] = originBlockedAngles
            --end
        end
        table.clear(originUpdates)
        --debug.profileend()
    end
    --debug.profileend()

    ----Debug aid:
    ----Set total cost to the be ratio between calculated total cost and actual distance (for reachable cells)
    --for index, currentCell in cells do
    --    local myPosition: Vector3 = currentCell[IDX_CELL_POSITION]
    --    local myOrigin: CellOrigin = currentCell[IDX_CELL_ORIGIN]
    --    local myCost = currentCell[IDX_CELL_TOTALCOST]
    --    if (myOrigin ~= INVALID_ORIGIN) and (myCost < EXTRA_BLOCKED_CELL_COST) then
    --        local actualCost = (myOrigin[IDX_ORIGIN_COST_TO_GOAL] + (myOrigin[IDX_ORIGIN_POSITION] - myPosition).Magnitude) / GRID_COORD_SPACING
    --        currentCell[IDX_CELL_TOTALCOST] = (myCost / actualCost) * 100 --estimate as percentage of actual
    --    end
    --end

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
local function IsLineTraversable(field: Field, start: Vector3, finish: Vector3): boolean
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
                if not cell or cell[IDX_CELL_BLOCKED] then
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
                if not cell or cell[IDX_CELL_BLOCKED] then
                    return false
                end
                cell = cells[index + indexDelta]
                if not cell or cell[IDX_CELL_BLOCKED] then

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
                if not cell or cell[IDX_CELL_BLOCKED] then
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
                if not cell or cell[IDX_CELL_BLOCKED] then
                    return false
                end
                cell = cells[index + indexDelta]
                if not cell or cell[IDX_CELL_BLOCKED] then
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
        if not cell or cell[IDX_CELL_BLOCKED] then
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
    if not cell or cell[IDX_CELL_BLOCKED] then
        return false
    end

    --Step line B between X and Z grid crossings, checking each cell along the field
    index = CoordToIndex(pBXGrid, pBZGrid)
    while (tMaxXB < 1) or (tMaxZB < 1) do
        cell = cells[index]
        if not cell or cell[IDX_CELL_BLOCKED] then
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
    if not cell or cell[IDX_CELL_BLOCKED] then
        return false
    end

    return true
end
FlowField.IsLineTraversable = IsLineTraversable


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

            if not cell or cell[IDX_CELL_BLOCKED] then
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

    return cell and not cell[IDX_CELL_BLOCKED] and (cell[IDX_CELL_TOTALCOST] < EXTRA_BLOCKED_CELL_COST)
end


--Calculate whether a point has an obstacle covering it
function FlowField.IsPointBlocked(field: Field, point: Vector3): boolean
    local cellPos = ToCellGridRound(point)
    local cell = field.Cells[CoordToIndex(cellPos.X, cellPos.Z)]

    return cell and cell[IDX_CELL_BLOCKED]
end


--Use bilinear interpolation to get the direction from a particular point
local function GetDirection(field: Field, position: Vector3): Vector3
    local cells = field.Cells
    local borderCells = field.BorderCells

    local upperLeft = ToCellGridFloor(position)
    local lowerRight = ToCellGridCeil(position)
    --local lowerLeft = Vector3.new(upperLeft.X, 0, lowerRight.Z)
    --local upperRight = Vector3.new(lowerRight.X, 0, upperLeft.Z)

    local delta = (position - upperLeft) / (lowerRight - upperLeft):Max(Vector3.one)

    local ulXIndex = XCoordToIndex(upperLeft.X)
    local ulZIndex = DeltaZCoordToIndex(upperLeft.Z)
    local lrXIndex = XCoordToIndex(lowerRight.X)
    local lrZIndex = DeltaZCoordToIndex(lowerRight.Z)

    --local llIndex = CoordToIndex(lowerLeft.X, lowerLeft.Z)
    --local urIndex = CoordToIndex(upperRight.X, upperRight.Z)
    --local ulIndex = CoordToIndex(upperLeft.X, upperLeft.Z)
    --local lrIndex = CoordToIndex(lowerRight.X, lowerRight.Z)

    local llIndex = ulXIndex + lrZIndex
    local urIndex = lrXIndex + ulZIndex
    local ulIndex = ulXIndex + ulZIndex
    local lrIndex = lrXIndex + lrZIndex

    local llCell = cells[llIndex] or borderCells[llIndex]
    local urCell = cells[urIndex] or borderCells[urIndex]
    local ulCell = cells[ulIndex] or borderCells[ulIndex]
    local lrCell = cells[lrIndex] or borderCells[lrIndex]

    --order doesn't matter but needs to be deterministic
    local backupOrder = table.create(4)

    local direction = Vector3.zero
    local directionDivisor = 0

    local deltaX = delta.X
    local deltaZ = delta.Z
    local oneMinusDeltaX = (1 - deltaX)
    local oneMinusDeltaZ = (1 - deltaZ)

    if llCell then
        local mul = oneMinusDeltaX * deltaZ
        direction += mul * llCell[IDX_CELL_DIRECTION]
        directionDivisor += mul
        table.insert(backupOrder, llCell)
    end
    if lrCell then
        local mul = deltaX * deltaZ
        direction += mul * lrCell[IDX_CELL_DIRECTION]
        directionDivisor += mul
        table.insert(backupOrder, lrCell)
    end
    if ulCell then
        local mul = oneMinusDeltaX * oneMinusDeltaZ
        direction += mul * ulCell[IDX_CELL_DIRECTION]
        directionDivisor += mul
        table.insert(backupOrder, ulCell)
    end
    if urCell then
        local mul = deltaX * oneMinusDeltaZ
        direction += mul * urCell[IDX_CELL_DIRECTION]
        directionDivisor += mul
        table.insert(backupOrder, urCell)
    end

    if (directionDivisor > 0) then
        direction /= directionDivisor
    end

    if direction == Vector3.zero then
        local bestCellDirectionCost = math.huge
        local bestCellDirection = Vector3.zero
        local bestCellPositionCost = math.huge
        local bestCellPosition = Vector3.zero
        for _, cell in backupOrder do
            local cellCost = cell[IDX_CELL_TOTALCOST]
            local cellDirection = cell[IDX_CELL_DIRECTION]
            if (cellDirection ~= Vector3.zero) and (cellCost < bestCellDirectionCost) then
                bestCellDirectionCost = cellCost
                bestCellDirection = cellDirection
            end
            local toCellDirection = cell[IDX_CELL_POSITION] - position
            if (toCellDirection ~= Vector3.zero) and (cellCost < bestCellPositionCost) then
                bestCellPositionCost = cellCost
                bestCellPosition = toCellDirection
            end
        end
        if (bestCellDirection ~= Vector3.zero) and (bestCellDirectionCost < bestCellPositionCost) then
            direction = bestCellDirection
        else--if bestCellPosition ~= Vector3.zero then
            direction = bestCellPosition - position
        end
    end
    if direction == Vector3.zero then
        error("Direction is zero!")
    end
    if (direction.X ~= direction.X) or (direction.Y ~= direction.Y) or (direction.Z ~= direction.Z) then
        error("NAN!")
    end

    return direction.Unit * GRID_COORD_SPACING
end
FlowField.GetDirection = GetDirection


return table.freeze(FlowField)