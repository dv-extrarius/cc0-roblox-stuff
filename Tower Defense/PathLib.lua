--!strict
local PathLib = {}

local GRID_COORD_SPACING = 2 --Path nodes are spaced every other stud
local GRID_COORD_OFFSET = 1  --Path nodes are on odd-numbered studs
local GRID_INDEX_MUL = 131072 --Chosen as 2^floor(52/3) for 52 mantissa bits to potentially fit 3 coordinates
export type PathNode = {Position: Vector3, Index: number, Neighbors: {PathNode}, RandomWeight: number, TotalCost: number, Blocked: boolean}
export type PathMesh = {Nodes: {[number]: PathNode}, IsFinalized: boolean}
export type PathList = {Vector3}

--Function to convert a coordinate to a node index
local function CoordToIndex(x: number, z: number): number
	return (x + (z * GRID_INDEX_MUL))
end
--Function to get a neighbor index from a given index
local function IndexDecreaseX(index: number): number
	return (index - GRID_COORD_SPACING)
end
local function IndexIncreaseX(index: number): number
	return index + GRID_COORD_SPACING
end
local function IndexDecreaseZ(index: number): number
	return index - (GRID_COORD_SPACING * GRID_INDEX_MUL)
end
local function IndexIncreaseZ(index: number): number
	return index + (GRID_COORD_SPACING * GRID_INDEX_MUL)
end


--Functions to move coordinates to pathing grid - odd-numbered studs
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
function PathLib.Clone(self: PathMesh): PathMesh
	local newMesh: PathMesh = {Nodes = {}, IsFinalized = false}
	local nodes = newMesh.Nodes

	for index, current in self.Nodes do
		nodes[index] = {
			Position = current.Position,
			Index = index,
			Neighbors = {},
			RandomWeight = current.RandomWeight,
			TotalCost = math.huge,
			Blocked = current.Blocked,
		}
	end

	if self.IsFinalized then
		PathLib.FinalizeMesh(newMesh)
	end
	return newMesh
end

--Add nodes to a mesh to cover a specified region
function PathLib.AddNodes(self: PathMesh, min: Vector3, max: Vector3, expand: boolean?): ()
	local nodes = self.Nodes

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
					Position = Vector3.new(x, 0, z),
					Index = index,
					Neighbors = {},
					RandomWeight = math.random() / 10000.0,
					TotalCost = math.huge,
					Blocked = false,
				}
			end
			index = IndexIncreaseX(index)
		end
	end
end

--Do final processing on each node in the mesh and freeze it
function PathLib.FinalizeMesh(self: PathMesh): ()
	local nodes = self.Nodes
	local neighbor: PathNode?

	--Connect each node to 4-way neighbors if they exist
	for index, current in nodes do
		neighbor = nodes[IndexDecreaseX(index)]
		if neighbor ~= nil then
			table.insert(current.Neighbors, neighbor)
		end

		neighbor = nodes[IndexIncreaseX(index)]
		if neighbor ~= nil then
			table.insert(current.Neighbors, neighbor)
		end

		neighbor = nodes[IndexDecreaseZ(index)]
		if neighbor ~= nil then
			table.insert(current.Neighbors, neighbor)
		end

		neighbor = nodes[IndexIncreaseZ(index)]
		if neighbor ~= nil then
			table.insert(current.Neighbors, neighbor)
		end

		table.freeze(current.Neighbors)
	end

	self.IsFinalized = true
	table.freeze(self.Nodes)
end

--Mark all nodes in the specified region as blocked
function PathLib.BlockNodes(self: PathMesh, min: Vector3, max: Vector3, expand: boolean?): ()
	local nodes = self.Nodes

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
				node.Blocked = true
			end
			index = IndexIncreaseX(index)
		end
	end
end

--Mark all nodes in the specified region as unblocked
function PathLib.UnblockNodes(self: PathMesh, min: Vector3, max: Vector3, expand: boolean?): ()
	local nodes = self.Nodes

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
				node.Blocked = false
			end
			index = IndexIncreaseX(index)
		end
	end
end

--Find a path between the given locations using the path mesh
function PathLib.FindPath(self: PathMesh, start: Vector3, finish: Vector3): PathList?
	--Look up the start and finish locations in the node grid
	start = PathLib.ToPathGridRound(start)
	finish = PathLib.ToPathGridRound(finish)

	local startIndex = CoordToIndex(start.X, start.Z)
	local startNode = self.Nodes[startIndex]
	local finishNode = self.Nodes[CoordToIndex(finish.X, finish.Z)]

	if startNode == nil then
		warn("Start of (", start, ") is outside path nodes")
		return nil
	end
	if finishNode == nil then
		warn("Finish of (", finish, ") is outside path nodes")
		return nil
	end

	--Reset costs to maximum value
	for _, node in self.Nodes do
		node.TotalCost = math.huge
	end
	--Set initial node to no cost
	startNode.TotalCost = 0

	local frontWave: {[number]: PathNode} = {}
	local nextWave: {[number]: PathNode} = {}

	--Do a full expansion of nodes in waves until all unblocked nodes have been assigned the minimum cost to get there
	frontWave[startIndex] = startNode
	while next(frontWave) ~= nil do
		for _, current in frontWave do
			local TotalCost = current.TotalCost + 1 + current.RandomWeight

			for _, neighbor in current.Neighbors do
				if (not neighbor.Blocked) and (TotalCost < neighbor.TotalCost) then
					neighbor.TotalCost = TotalCost
					nextWave[neighbor.Index] = neighbor
				end
			end
		end
		frontWave, nextWave = nextWave, frontWave
		table.clear(nextWave)
	end

	--If the finish node wasn't visited, no path can exist
	if finishNode.TotalCost == math.huge then
		return nil
	end

	--Follow the cheapest nodes from finish to start
	local path: {PathNode} = {finishNode}
	while path[#path].Position ~= startNode.Position do
		local current = path[#path]
		local bestNeighbor: PathNode? = nil

		for _, neighbor in current.Neighbors do
			if (not neighbor.Blocked) and ((bestNeighbor == nil) or (neighbor.TotalCost < bestNeighbor.TotalCost)) then
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

	--If the path didn't connect to the start node, a path doesn't exist
	if path[#path].Position ~= startNode.Position then
		return nil
	end

	--Store the position of each node used in the correct (reversed, so start to finish) order
	local pathLen = #path
	local result: PathList = table.create(pathLen)

	for i = 1, pathLen do
		result[i] = path[pathLen - i + 1].Position
	end

	return result
end

return table.freeze(PathLib)