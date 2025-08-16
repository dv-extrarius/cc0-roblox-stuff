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
local IDX_CELL_BLOCKEDSTATE = 6
local IDX_CELL_ORIGIN = 7
--local IDX_CELL_DEBUG = 8
local IDX_CELLBORDER_IDSTRING = 8 --This index MUST NOT EXIST in normal cells

type FieldCell = {any} --Unfortuantely heterogeneous arrays not yet supported in type system
type FieldCellSet = {[FieldCell]: true}
--[[
type FieldCell = {
    Neighbors: {FieldCell},
    Position: Vector3,
    Index: number,
    TotalCost: number,
    Blocked: boolean,
    Origin: CellOrigin,
}
]]
type AngleRange = Vector3 --X is start and Y is end (counterclockwise)

local IDX_ORIGIN_POSITION = 1
local IDX_ORIGIN_COST_TO_GOAL = 2
local IDX_ORIGIN_BLOCKED_ANGLES = 3
type CellOrigin = {any} --Unfortuantely heterogeneous arrays not yet supported in type system
--[[
type CellOrigin = {
    Position: Vector3,
    CostToGoal: number,
    BlockedAngles: {AngleRange},
}
]]
export type Field = {
    Cells: {[number]: FieldCell},
    BorderCells: {[number]: FieldCell},
    SortedCells: {FieldCell},
    IsFinalized: boolean,
    GoalCells: {FieldCell},

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
--local GRID_SPACING_VEC3 = Vector3.new(GRID_COORD_SPACING, 0, GRID_COORD_SPACING)
--local GRID_OFFSET_VEC3 = Vector3.new(GRID_COORD_OFFSET, 0, GRID_COORD_OFFSET)
--local GRID_SPACING_INVERSE_VEC3 = Vector3.new(1/GRID_COORD_SPACING, 0, 1/GRID_COORD_SPACING)
local TAU = 2 * math.pi
local EXTRA_BLOCKED_CELL_COST = 2^24 --arbitrary large value much larger than the number of cells
local INVALID_ORIGIN: CellOrigin = table.freeze({
    Vector3.zero,                                      --Position
    math.huge,                                         --CostToGoal
    table.freeze({Vector3.new(-math.pi, math.pi, 0)}), --BlockedAngles
} :: CellOrigin)

local BLOCKEDSTATE_NEG_X = 1
local BLOCKEDSTATE_POS_X = 2
local BLOCKEDSTATE_NEG_Z = 4
local BLOCKEDSTATE_POS_Z = 8

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
local COORD_INDEX_BASE = (GRID_INDEX_MUL / 2) + ((GRID_INDEX_MUL / 2) * GRID_INDEX_MUL)
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
        EnableDebugMessages = config.EnableDebugMessages and true or false --cast to boolean
    end
end


--Create an empty field
function FlowField.NewField(): Field
    return {
        Cells = {},
        BorderCells = {},
        SortedCells = {},
        IsFinalized = false,
        GoalCells = {}
    }
end


--Clone a field. Finalizes the cells if the original is finialized
function FlowField.Clone(oldField: Field, skipFinalize: boolean?): Field
    local newField: Field = {
        Cells = {},
        BorderCells = {},
        SortedCells = table.create(#oldField.SortedCells),
        IsFinalized = false,
        GoalCells = table.create(#oldField.GoalCells),
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
            --newCell[IDX_CELL_DEBUG] = {}

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
                    table.create(4),       --Neighbors      IDX_CELL_NEIGHBORS
                    Vector3.new(x, 0, z),  --Position       IDX_CELL_POSITION
                    index,                 --Index          IDX_CELL_INDEX
                    math.huge,             --TotalCost      IDX_CELL_TOTALCOST
                    false,                --Blocked        IDX_CELL_BLOCKED
                    0,                     --BlockedState   IDX_CELL_BLOCKEDSTATE
                    INVALID_ORIGIN,        --Origin         IDX_CELL_ORIGIN
                    --{},                    --Debug          IDX_CELL_DEBUG
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
        frozenEmptyNeighbors, --Neighbors      IDX_CELL_NEIGHBORS
        borderPosition,       --Position       IDX_CELL_POSITION
        neighborIndex,        --Index          IDX_CELL_INDEX
        math.huge,            --TotalCost      IDX_CELL_TOTALCOST
        true,                 --Blocked        IDX_CELL_BLOCKED
        0,                    --BlockedState   IDX_CELL_BLOCKEDSTATE
        INVALID_ORIGIN,       --Origin         IDX_CELL_ORIGIN
        --{},                   --Debug          IDX_CELL_DEBUG
        borderCellStr         --IDX_CELLBORDER_IDSTRING
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
    table.sort(sorted, function (lhs, rhs) return lhs[IDX_CELL_INDEX] < rhs[IDX_CELL_INDEX] end)

    --Connect each cell to 4-way neighbors if they exist and create border cells if they don't
    for index, currentCell in cells do
        local neighbors: {FieldCell} = currentCell[IDX_CELL_NEIGHBORS]
        local currentPosition: Vector3 = currentCell[IDX_CELL_POSITION]

        neighborIndex = index + DeltaXCoordToIndex(-GRID_COORD_SPACING)
        neighbor = cells[neighborIndex] or borderCells[neighborIndex]
        if not neighbor then
            neighbor = MakeBorderCell(neighborIndex, currentPosition - GRID_COORD_SPACING*Vector3.xAxis)
            borderCells[neighborIndex] = neighbor
        end
        table.insert(neighbors, neighbor :: FieldCell)


        neighborIndex = index + DeltaXCoordToIndex(GRID_COORD_SPACING)
        neighbor = cells[neighborIndex] or borderCells[neighborIndex]
        if not neighbor then
            neighbor = MakeBorderCell(neighborIndex, currentPosition + GRID_COORD_SPACING*Vector3.xAxis)
            borderCells[neighborIndex] = neighbor
        end
        table.insert(neighbors, neighbor :: FieldCell)


        neighborIndex = index + DeltaZCoordToIndex(-GRID_COORD_SPACING)
        neighbor = cells[neighborIndex] or borderCells[neighborIndex]
        if not neighbor then
            neighbor = MakeBorderCell(neighborIndex, currentPosition - GRID_COORD_SPACING*Vector3.zAxis)
            borderCells[neighborIndex] = neighbor
        end
        table.insert(neighbors, neighbor :: FieldCell)


        neighborIndex = index + DeltaZCoordToIndex(GRID_COORD_SPACING)
        neighbor = cells[neighborIndex] or borderCells[neighborIndex]
        if not neighbor then
            neighbor = MakeBorderCell(neighborIndex, currentPosition + GRID_COORD_SPACING*Vector3.zAxis)
            borderCells[neighborIndex] = neighbor
        end
        table.insert(neighbors, neighbor :: FieldCell)


        table.freeze(neighbors)
    end

    field.IsFinalized = true
    table.freeze(cells)
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

local angleRangesCache = table.create(1)
--Create a new angle range from an existin range and a new range. Steals the input list for cache use
local function AddAngleToAngleRange(ranges: {AngleRange}, addRange: AngleRange): {AngleRange}
    --If the angle straddles the wrap from +180 to -180, we have to break it up into two angle ranges
    if addRange.X > addRange.Y then
        ranges = AddAngleToAngleRange(ranges, Vector3.new(addRange.X, math.pi, 0))
        addRange = Vector3.new(-math.pi, addRange.Y, 0)
    end

    local numRanges = #ranges
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


--Calculate the angle range occluded by a square from a given origin, expanded to account for "unit" diameter equal to grid space
local function SquareAnglesExpanded(origin: Vector3, squareCenter: Vector3): (number, number)
    local halfGridSpacing = 0.5 * GRID_COORD_SPACING
    local halfUnitRadius = halfGridSpacing

    local delta = squareCenter - origin

    local xCenter = delta.X
    local yCenter = delta.Z
    --Center adjusted to be in quadruant 1
    local absXCenter = math.abs(xCenter)
    local absYCenter = math.abs(yCenter)
    --Perpendicular offsets to account for object radius
    local du = delta.Unit
    local perpX = math.abs(du.Z) * halfUnitRadius
    local perpY = -math.abs(du.X) * halfUnitRadius

    --Corners Offsets from Center: (1) Left Top = -X,+Y;  (2) Left Bottom = -X,-Y; (3) Right Bottom = +X,-Y
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
        {nil},           --BlockedAngles (faster than table.create(1))
    }
    return origin
end

--"Magic" type that makes setting wave information easier - it automatically creates a new wave as needed
type Set<K> = {[K]: true}
type SetDictBase<K,V> = {[K]: Set<V>}
type SetDictMeta<K,V> = {__index: (SetDict<K,V>, K) -> Set<V>, __iter: unknown?}
type SetDict<K, V> = typeof(setmetatable({} :: SetDictBase<K,V>, {} :: SetDictMeta<K,V>))
local function SetDictIndexer<K, V>(self: SetDict<K, V>, key: K): Set<V>
    local value: Set<V> = {}
    self[key] = value
    return value
end
type WaveDict = SetDict<number, FieldCell>
local WaveDictMeta: SetDictMeta<number, FieldCell> = {__index = SetDictIndexer}
local function NewWaveDict(): WaveDict
    return setmetatable({}, WaveDictMeta)
end


--local updateDebugData: {Instance} = {}
--Update the costs so the field points the way to the goal
local function UpdateField(field: Field, goalCells: {FieldCell}): boolean
    --for _, vis in updateDebugData do
    --    vis:Destroy()
    --end
    --table.clear(updateDebugData)

    --Updating the costs requires a finalized field
    if not field.IsFinalized then
        CustomError(1, "Attempted UpdateField without finalizing the field.")
        return false
    end

    local cells = field.Cells

    --Reset costs to maximum value
    for _, cell in cells do
        cell[IDX_CELL_TOTALCOST] = math.huge
        cell[IDX_CELL_ORIGIN] = INVALID_ORIGIN
        local blockedState = 0
        --For unblocked cells, record which directions are blocked
        if not cell[IDX_CELL_BLOCKED] then
            --debug.profilebegin("BlockedState")
            local myPosition = cell[IDX_CELL_POSITION]
            local myPosX = myPosition.X
            local myPosZ = myPosition.Z
            for _, neighbor in cell[IDX_CELL_NEIGHBORS] do
                if not neighbor[IDX_CELL_BLOCKED] or neighbor[IDX_CELLBORDER_IDSTRING] then
                    continue
                end
                local neighborPosition = neighbor[IDX_CELL_POSITION]
                local neighborPosX = neighborPosition.X
                local neighborPosZ = neighborPosition.Z
                if myPosX < neighborPosX then
                    blockedState = blockedState + BLOCKEDSTATE_NEG_X
                elseif myPosX > neighborPosX then
                    blockedState = blockedState + BLOCKEDSTATE_POS_X
                elseif myPosZ < neighborPosZ then
                    blockedState = blockedState + BLOCKEDSTATE_NEG_Z
                elseif myPosZ > neighborPosZ then
                    blockedState = blockedState + BLOCKEDSTATE_POS_Z
                end
            end
            --debug.profileend()
        end
        cell[IDX_CELL_BLOCKEDSTATE] = blockedState
        --table.clear(cell[IDX_CELL_DEBUG])
    end

    --No need to clone the goal table, only place it comes from is where we create it
    field.GoalCells = goalCells

    --Do a full expansion of cells in waves until all unblocked cells have been assigned the minimum cost to get there
    local allWaves: WaveDict = NewWaveDict()
    local currentWaveNum = 0

    --Info used to update angle ranges for origins
    local originUpdates: {[CellOrigin]: {[FieldCell]: Vector3}} = {}
    local originCache: {[Vector3]: CellOrigin} = {}

    --Initialize goal cells to 0 distance and set the first wave to contain their neighbors
    for _, goalCell in goalCells do
        goalCell[IDX_CELL_TOTALCOST] = 0
        local originPosition = goalCell[IDX_CELL_POSITION]
        local origin = MakeOrigin(originPosition, 0)
        originCache[originPosition] = origin

        goalCell[IDX_CELL_ORIGIN] = origin
        --originUpdates[origin] = {}
        allWaves[0][goalCell] = true
    end

    --local numCellsExamined = 0
    local numOriginsCreated = #goalCells
    repeat
        local initialCurrentWaveNum = currentWaveNum
        local currentWave: FieldCellSet = rawget(allWaves :: any, currentWaveNum)
        --if the wave number we expected doesn't exist, find the next lowest wave number
        if not currentWave then
            --Usually the next wave number will be pretty close, so try a few
            for _ = 1, 2*GRID_COORD_SPACING do
                currentWaveNum += 1
                currentWave = rawget(allWaves :: any, currentWaveNum)
                if currentWave then
                    break
                end
            end
            --If a few steps didn't find anything, manually find the next lowest
            if not currentWave then
                currentWaveNum = math.huge
                for waveNum, wave in allWaves do
                    if waveNum < currentWaveNum then
                        currentWaveNum = waveNum
                        currentWave = wave
                    end
                end
            end
            if not currentWave then
                break
            end
        end
        allWaves[currentWaveNum] = nil
        --[[
        do
            local lowestWaveNum = math.huge
            for waveNum, wave in allWaves do
                if waveNum < lowestWaveNum then
                    lowestWaveNum = waveNum
                end
            end
            if lowestWaveNum < currentWaveNum then
                local numWaves = 0
                for _ in allWaves do
                    numWaves += 1
                end
                print("currentWaveNum too high! current", currentWaveNum, " vs lowest", lowestWaveNum, "(initial =", initialCurrentWaveNum, "); numWaves =", numWaves)
            end
        end
        --]]

        --Exhaust the current wave to update cost of all reachable areas
        for currentCell, _ in currentWave do
            --numCellsExamined += 1
            --table.insert(currentCell[IDX_CELL_DEBUG], numCellsExamined)

            local myCost: number = currentCell[IDX_CELL_TOTALCOST]
            local myPosition: Vector3 = currentCell[IDX_CELL_POSITION]
            local myOrigin: CellOrigin = currentCell[IDX_CELL_ORIGIN]
            --If this cell doesn't have an origin, one must be made to the lowest cost neighbor
            if myOrigin == INVALID_ORIGIN then
                --debug.profilebegin("MakeNewOrigin")
                local myIsBlocked: boolean = currentCell[IDX_CELL_BLOCKED]
                local bestPosition = myPosition
                local bestPositionCost = math.huge
                for _, neighborCell in currentCell[IDX_CELL_NEIGHBORS] do
                    local neighborOrigin = neighborCell[IDX_CELL_ORIGIN]
                    if neighborOrigin == INVALID_ORIGIN then
                        continue
                    end
                    local neighborCost = neighborCell[IDX_CELL_TOTALCOST]
                    local neighborPosition = neighborCell[IDX_CELL_POSITION]
                    if neighborCost < bestPositionCost then
                        bestPositionCost = neighborCost
                        bestPosition = neighborPosition
                    end
                end
                if bestPositionCost == math.huge then
                    print("Visited cell has no valid neighbors!")
                end
                if myPosition:FuzzyEq(bestPosition) then
                    print("Origin at self!?")
                end
                local newOrigin = originCache[bestPosition]
                if not newOrigin then
                    for tempPosition, _ in originCache do
                        if (tempPosition - bestPosition).Magnitude < 1 then
                            print()
                            break
                        end
                    end
                    newOrigin = MakeOrigin(bestPosition, bestPositionCost)
                    --originUpdates[newOrigin] = {}
                    numOriginsCreated += 1
                    originCache[bestPosition] = newOrigin
                end
                myOrigin = newOrigin
                local tempPositionCost = myOrigin[IDX_ORIGIN_COST_TO_GOAL]
                local newCost = tempPositionCost + (bestPosition - myPosition).Magnitude
                if myIsBlocked then
                    newCost += EXTRA_BLOCKED_CELL_COST
                end
                currentCell[IDX_CELL_ORIGIN] = myOrigin
                currentCell[IDX_CELL_TOTALCOST] = newCost
                if myCost + 1 < newCost then
                    local newWaveNum = newCost // 1
                    allWaves[newWaveNum][currentCell] = true
                    if newWaveNum < currentWaveNum then
                        currentWaveNum = math.min(currentWaveNum, newWaveNum)
                    end
                    continue
                end
                myCost = newCost
                --debug.profileend()
            end

            local myOriginCost: number = myOrigin[IDX_ORIGIN_COST_TO_GOAL]
            local myOriginPosition: Vector3 = myOrigin[IDX_ORIGIN_POSITION]
            if myCost < myOriginCost + (myOriginPosition - myPosition).Magnitude then
                print("!?!?!")
            end
            local myOriginBlockedAngles: {AngleRange} = myOrigin[IDX_ORIGIN_BLOCKED_ANGLES]
            local myOriginUpdates = originUpdates[myOrigin]
            if not myOriginUpdates then
                myOriginUpdates = {}
                originUpdates[myOrigin] = myOriginUpdates
            end
            for _, neighborCell in currentCell[IDX_CELL_NEIGHBORS] do
                --Border cells need to be added as blockers but not processed further
                if neighborCell[IDX_CELLBORDER_IDSTRING] then
                    myOriginUpdates[neighborCell] = neighborCell[IDX_CELL_POSITION]
                    continue
                end
                local neighborOrigin = neighborCell[IDX_CELL_ORIGIN]
                --If the neighbor already has the same origin, no need to check anything
                if neighborOrigin == myOrigin then
                    continue
                end
                local neighborIsBlocked: boolean = neighborCell[IDX_CELL_BLOCKED]
                local neighborPosition = neighborCell[IDX_CELL_POSITION]
                if neighborIsBlocked then
                    myOriginUpdates[neighborCell] = neighborPosition
                end
                local neighborCost = neighborCell[IDX_CELL_TOTALCOST]
                local deltaFromOrigin = neighborPosition - myOriginPosition
                local newCost = myOriginCost + deltaFromOrigin.Magnitude
                if neighborIsBlocked then
                    newCost += EXTRA_BLOCKED_CELL_COST
                end
                --If changing origins would increase the cost, no reason to continue
                if newCost >= neighborCost then
                    --See if creating a new origin is still cheaper
                    local deltaFromNewOrigin = neighborPosition - myPosition -- =GRID_COORD_SPACING
                    newCost = myCost + deltaFromNewOrigin.Magnitude
                    if neighborIsBlocked then
                        newCost += EXTRA_BLOCKED_CELL_COST
                    end
                    if newCost < neighborCost then
                        print("?!?!?")
                    end
                    continue
                end
                local blockedState = neighborCell[IDX_CELL_BLOCKEDSTATE]
                local blockerMakesInvalid = false
                local deltaFromOriginX = deltaFromOrigin.X
                local deltaFromOriginZ = deltaFromOrigin.Z
                if not neighborIsBlocked then
                    if bit32.btest(blockedState, BLOCKEDSTATE_NEG_X) and (deltaFromOriginX < 0) then
                        blockerMakesInvalid = true
                    elseif bit32.btest(blockedState, BLOCKEDSTATE_POS_X) and (deltaFromOriginX > 0) then
                        blockerMakesInvalid = true
                    elseif bit32.btest(blockedState, BLOCKEDSTATE_NEG_Z) and (deltaFromOriginZ < 0) then
                        blockerMakesInvalid = true
                    elseif bit32.btest(blockedState, BLOCKEDSTATE_POS_Z) and (deltaFromOriginZ > 0) then
                        blockerMakesInvalid = true
                    end
                end
                local checkAngle = math.atan2(deltaFromOriginZ, deltaFromOriginX)
                --Is this a blocked cell or can it reach my origin?
                if not blockerMakesInvalid and not DoesAngleRangeContainAngle(myOriginBlockedAngles, checkAngle) then
                    neighborCell[IDX_CELL_ORIGIN] = myOrigin
                    neighborCell[IDX_CELL_TOTALCOST] = newCost
                    --Unschedule previous versions
                    if neighborCost ~= math.huge then
                        allWaves[neighborCost // 1][neighborCell] = nil
                    end
                    --Schedule this cell to be expanded according to its cost
                    local neighborWaveNum = newCost // 1
                    allWaves[neighborWaveNum][neighborCell] = true
                    if neighborWaveNum < currentWaveNum then
                        currentWaveNum = math.min(currentWaveNum, neighborWaveNum)
                    end
                else
                    --See if creating a new origin is still cheaper (and schedule that if so)
                    local deltaFromNewOrigin = neighborPosition - myPosition -- =GRID_COORD_SPACING
                    newCost = myCost + deltaFromNewOrigin.Magnitude
                    if neighborIsBlocked then
                        newCost += EXTRA_BLOCKED_CELL_COST
                    end
                    if newCost < neighborCost then
                        neighborCell[IDX_CELL_ORIGIN] = INVALID_ORIGIN
                        neighborCell[IDX_CELL_TOTALCOST] = newCost
                        --Unschedule previous versions
                        if neighborCost ~= math.huge then
                            allWaves[neighborCost // 1][neighborCell] = nil
                        end
                        --Schedule this cell to be expanded according to its cost
                        local neighborWaveNum = newCost // 1
                        allWaves[neighborWaveNum][neighborCell] = true
                        if neighborWaveNum < currentWaveNum then
                            currentWaveNum = math.min(currentWaveNum, neighborWaveNum)
                        end
                    end
                end
            end
        end

        --Apply all updates to origins
        --debug.profilebegin("UpdateOriginAngles")
        for origin, blockedCellPositions in originUpdates do
            local originPosition = origin[IDX_ORIGIN_POSITION]
            local originBlockedAngles = origin[IDX_ORIGIN_BLOCKED_ANGLES]
            for blockingCell, blockedCellPosition in blockedCellPositions do
                local startAngle, endAngle = SquareAnglesExpanded(originPosition, blockedCellPosition)
                originBlockedAngles = AddAngleToAngleRange(originBlockedAngles, Vector3.new(startAngle, endAngle, 0))
            end
            origin[IDX_ORIGIN_BLOCKED_ANGLES] = originBlockedAngles
        end
        table.clear(originUpdates)
        --debug.profileend()
    until not next(allWaves)
    --[[
    local usedOrigins: {[CellOrigin]: true} = {}
    for index, cell in cells do
        usedOrigins[cell[IDX_CELL_ORIGIN] ] = true
    end
    local sortedOrigins = {}

    for position, origin in originCache do
        table.insert(sortedOrigins, origin)
    end
    table.sort(sortedOrigins, function(a, b) return a[IDX_ORIGIN_COST_TO_GOAL] < b[IDX_ORIGIN_COST_TO_GOAL] end)
    for _, origin in sortedOrigins do
        local used = usedOrigins[origin]
        local position = origin[IDX_ORIGIN_POSITION]
        --print("<", position, ">", (if used then "used" else "forgotten"), " @ ", origin[IDX_ORIGIN_COST_TO_GOAL])
        if not used then
            DebugHelp.SaveInsances(updateDebugData, DebugHelp.MakeBall(position + 2.5*Vector3.yAxis, {Radius = 2.5, Color = Color3.new(0, 0, 0.5)}))
        end
    end
    --]]

    --print("numCellsExamined =", numCellsExamined, "; numOriginsCreated =", numOriginsCreated)
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

    local invDeltaX = 1 / deltaX
    local invDeltaZ = 1 / deltaZ
    --The T values at which a grid crossing takes place
    local halfStepX = stepX * 0.5
    local halfStepZ = stepZ * 0.5
    local tMaxXA = (pAXGrid + halfStepX - pAX) * invDeltaX
    local tMaxZA = (pAZGrid + halfStepZ - pAZ) * invDeltaZ
    local tMaxXB = (pBXGrid + halfStepX - pBX) * invDeltaX
    local tMaxZB = (pBZGrid + halfStepZ - pBZ) * invDeltaZ

    --The change in T a step in each direction adavances.
    local tDeltaX = stepX * invDeltaX
    local tDeltaZ = stepZ * invDeltaZ

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


--Return the target point a unit at the given coordinate sould go towards
local function GetTarget(field: Field, position: Vector3): Vector3
    local cells = field.Cells
    do
        local cellPosition = ToCellGridRound(position)
        local index = CoordToIndex(cellPosition.X, cellPosition.Z)
        local cell: FieldCell? = cells[index]

        if cell then
            local origin = cell[IDX_CELL_ORIGIN]
            if origin ~= INVALID_ORIGIN then
                return origin[IDX_ORIGIN_POSITION]
            else
                print("Invalid Origin")
            end
        end
    end

    local bestGoalPosition: Vector3 = position
    local bestDistance = math.huge
    for _, goalCell in field.GoalCells do
        local goalPosition = goalCell[IDX_CELL_POSITION]
        local distance = (goalPosition - position).Magnitude
        if distance < bestDistance then
            bestDistance = distance
            bestGoalPosition = goalPosition
        end
    end
    return bestGoalPosition
end
FlowField.GetTarget = GetTarget


return table.freeze(FlowField)