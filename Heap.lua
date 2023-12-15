--[[
Copyright Extrarius 2023.
This work is marked with CC0 1.0 Universal.
To view a copy of this license, visit http://creativecommons.org/publicdomain/zero/1.0
--
This is an implementation of a pairing min-heap, which has constant time insertion.
Algorithms implemented from https://en.wikipedia.org/wiki/Pairing_heap
The algorithms were modified to create fewer temporaries, thus reducing garbage generated.
]]

--!strict

--The base node type used to build the heap
export type HeapNode = {value: any?, priority: number, subheaps: {HeapNode}}
export type Heap = HeapNode

--The type this module creates
type HeapLib = {
	__index: HeapLib,
	new: () -> Heap,
	push: (self: Heap, priority: number, value: any) -> (),
	peek: (self: Heap) -> (any, number),
	isEmpty: (self: Heap) -> boolean,
	pop: (self: Heap) -> (any, number),
	_Test: () -> ()
}

local HeapLib: HeapLib = {} :: HeapLib
HeapLib.__index = HeapLib

--Construct a new empty Heap
function HeapLib.new(): Heap
	return {
		value = nil,
		priority = math.huge,
		subheaps = {}
	} :: HeapNode
end

--Insert an element
function HeapLib.push(self: Heap, priority: number, value: any)
	if self.value == nil then
		self.value = value
		self.priority = priority
		self.subheaps = {}
	elseif self.priority < priority then
		table.insert(self.subheaps, {
			value = value,
			priority = priority,
			subheaps = {}
		} :: HeapNode)
	else
		table.insert(self.subheaps, {
			value = self.value,
			priority = self.priority,
			subheaps = {}
		} :: HeapNode)
		self.value = value
		self.priority = priority
	end
end

--peek at the lowest priority element
function HeapLib.peek(self: Heap): (any, number)
	return self.value, self.priority
end

--Test whether a heap is empty
function HeapLib.isEmpty(self: Heap): boolean
	return (self.value == nil)
end

--Utility function used by pop to (destructively) meld two heaps into one new heap
local function _Meld(heap1: HeapNode, heap2: HeapNode): HeapNode
	if heap1.value == nil then
		return heap2
	elseif heap2.value == nil then
		return heap1
	elseif heap1.priority < heap2.priority then
		table.insert(heap1.subheaps, heap2)
		return heap1
	else
		table.insert(heap2.subheaps, heap1)
		return heap2
	end
end

--Utility function used by pop to (destructively) remerge sub-heaps
local function _mergePairs(list: {HeapNode}): HeapNode
	while #list > 1 do
		local listsize = #list
		local numresults = (listsize + 1) // 2
		local results = table.create(numresults)

		for ii = 2,listsize,2 do
			results[ii / 2] = _Meld(list[ii-1], list[ii])
		end

		if listsize % 2 == 1 then
			results[numresults] = list[listsize]
		end

		list = results
	end

	return list[1]
end

--Remove the lowest priority element and return the value and prority
function HeapLib.pop(self: Heap): (any, number)
	local value = self.value
	local priority = self.priority
	local subheaps = self.subheaps

	if (value ~= nil) and (#subheaps > 0) then
		local heap = _mergePairs(subheaps)
		self.value = heap.value
		self.priority = heap.priority
		self.subheaps = heap.subheaps
	else
		self.value = nil
		self.priority = math.huge
		self.subheaps = {}
	end

	return value, priority
end

function HeapLib._Test()
	local h = HeapLib.new()

	--Test popping empty is nil
	assert(HeapLib.pop(h) == nil)

	--Test next most basic scenario, 
	HeapLib.push(h, 0, "A")
	assert(HeapLib.pop(h) == "A")

	--All permutations of 2-element priorities
	HeapLib.push(h, 0, "A")
	HeapLib.push(h, 1, "B")
	assert(HeapLib.pop(h) == "A")
	assert(HeapLib.pop(h) == "B")

	HeapLib.push(h, 1, "A")
	HeapLib.push(h, 0, "B")
	assert(HeapLib.pop(h) == "B")
	assert(HeapLib.pop(h) == "A")

	--All permutations of 3-element priorities
	HeapLib.push(h, 0, "A")
	HeapLib.push(h, 1, "B")
	HeapLib.push(h, 2, "C")
	assert(HeapLib.pop(h) == "A")
	assert(HeapLib.pop(h) == "B")
	assert(HeapLib.pop(h) == "C")

	HeapLib.push(h, 0, "A")
	HeapLib.push(h, 2, "B")
	HeapLib.push(h, 1, "C")
	assert(HeapLib.pop(h) == "A")
	assert(HeapLib.pop(h) == "C")
	assert(HeapLib.pop(h) == "B")

	HeapLib.push(h, 1, "A")
	HeapLib.push(h, 0, "B")
	HeapLib.push(h, 2, "C")
	assert(HeapLib.pop(h) == "B")
	assert(HeapLib.pop(h) == "A")
	assert(HeapLib.pop(h) == "C")

	HeapLib.push(h, 2, "A")
	HeapLib.push(h, 0, "B")
	HeapLib.push(h, 1, "C")
	assert(HeapLib.pop(h) == "B")
	assert(HeapLib.pop(h) == "C")
	assert(HeapLib.pop(h) == "A")

	HeapLib.push(h, 1, "A")
	HeapLib.push(h, 2, "B")
	HeapLib.push(h, 0, "C")
	assert(HeapLib.pop(h) == "C")
	assert(HeapLib.pop(h) == "A")
	assert(HeapLib.pop(h) == "B")

	HeapLib.push(h, 2, "A")
	HeapLib.push(h, 1, "B")
	HeapLib.push(h, 0, "C")
	assert(HeapLib.pop(h) == "C")
	assert(HeapLib.pop(h) == "B")
	assert(HeapLib.pop(h) == "A")

	--Test pushing more elements out of order
	HeapLib.push(h, 10, "A")
	HeapLib.push(h, 5, "B")
	HeapLib.push(h, 7, "C")
	HeapLib.push(h, 15, "D")
	HeapLib.push(h, 1, "E")
	assert(HeapLib.pop(h) == "E")
	assert(HeapLib.pop(h) == "B")
	assert(HeapLib.pop(h) == "C")
	assert(HeapLib.pop(h) == "A")
	assert(HeapLib.pop(h) == "D")

	--Test pushing non-empty heap
	HeapLib.push(h, 10, "A")
	HeapLib.push(h, 5, "B")
	HeapLib.push(h, 7, "C")
	HeapLib.push(h, 15, "D")
	assert(HeapLib.pop(h) == "B")
	assert(HeapLib.pop(h) == "C")
	HeapLib.push(h, 6, "E")
	HeapLib.push(h, 1, "F")
	HeapLib.push(h, 20, "G")
	assert(HeapLib.pop(h) == "F")
	assert(HeapLib.pop(h) == "E")
	assert(HeapLib.pop(h) == "A")
	assert(HeapLib.pop(h) == "D")
	assert(HeapLib.pop(h) == "G")

	--Test popping empty is still nil
	assert(HeapLib.pop(h) == nil)

	--Test interspersing pops and pushes on a larger value set (values 100-199)
	local TestValues: {number} = {
		112,170,166,151,180,136,145,105,138,176,135,120,148,194,161,125,143,164,189,171,
		169,104,103,153,130,122,162,117,121,137,126,152,149,116,146,150,177,178,191,119,
		127,133,159,179,157,186,185,192,183,111,158,197,196,155,174,109,128,160,106,165,
		199,141,190,142,188,175,110,173,107,195,163,193,167,147,132,123,139,129,156,172,
		198,114,118,100,102,140,184,182,134,113,144,101,187,108,115,131,168,124,154,181
	}

	local TestTracker: {number} = {}

	local function SliceArray<T>(arr: {T}, start: number, finish: number): {T}
		return table.move(arr, start, finish, 1, table.create(finish - start + 1))
	end

	local function MultiPush(start: number, finish: number)
		for _, val in ipairs(SliceArray(TestValues, start, finish)) do
			HeapLib.push(h, val, val)
			table.insert(TestTracker, val)
		end
	end

	local function MultiPop(count: number)
		table.sort(TestTracker, function(a: number, b: number): boolean return a < b end)
		for i = 1, count do
			assert(HeapLib.pop(h) == TestTracker[i])
		end
		local temp = SliceArray(TestTracker, count + 1, #TestTracker)
		TestTracker = temp
	end

	MultiPush(1, 10)
	MultiPop(1)
	MultiPush(11, 20)
	MultiPop(2)
	MultiPush(21, 30)
	MultiPop(3)
	MultiPush(31, 40)
	MultiPop(5)
	MultiPush(41, 50)
	MultiPop(7)
	MultiPush(51, 60)
	MultiPop(11)
	MultiPush(61, 70)
	MultiPop(13)
	MultiPush(71, 80)
	MultiPop(17)
	MultiPush(81, 90)
	MultiPop(19)
	MultiPush(91, 100)
	MultiPop(#TestTracker)

	--Test popping empty is still nil
	assert(HeapLib.pop(h) == nil)

	--Test completed
	print("Heap test succeeded")
end

table.freeze(HeapLib)
return HeapLib